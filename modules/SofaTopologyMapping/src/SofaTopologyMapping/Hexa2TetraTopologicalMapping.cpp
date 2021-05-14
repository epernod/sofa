/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaTopologyMapping/Hexa2TetraTopologicalMapping.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/ObjectFactory.h>

#include <SofaBaseTopology/HexahedronSetTopologyContainer.h>
#include <SofaBaseTopology/HexahedronSetTopologyModifier.h>

#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyModifier.h>

#include <sofa/core/topology/TopologyChange.h>
#include <SofaBaseTopology/GridTopology.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/defaulttype/Vec.h>
#include <map>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa::component::topology
{

using namespace sofa::defaulttype;

using namespace sofa::component::topology;
using namespace sofa::core::topology;

// Register in the Factory
int Hexa2TetraTopologicalMappingClass = core::RegisterObject("Special case of mapping where HexahedronSetTopology is converted to TetrahedronSetTopology")
        .add< Hexa2TetraTopologicalMapping >()

        ;

// Implementation

Hexa2TetraTopologicalMapping::Hexa2TetraTopologicalMapping()
    : swapping(initData(&swapping, false, "swapping","Boolean enabling to swapp hexa-edges\n in order to avoid bias effect"))
{
}

Hexa2TetraTopologicalMapping::~Hexa2TetraTopologicalMapping()
{
}

void Hexa2TetraTopologicalMapping::init()
{    
    // INITIALISATION of TETRAHEDRAL mesh from HEXAHEDRAL mesh :

    // recheck models
    bool modelsOk = true;
    if (!fromModel)
    {
        msg_error() << "No input mesh topology found. Consider setting the '" << fromModel.getName() << "' data attribute.";
        modelsOk = false;
    }

    if (!toModel)
    {
        msg_error() << "No output mesh topology found. Consider setting the '" << toModel.getName() << "' data attribute.";
        modelsOk = false;
    }


    // Making sure the output topology is derived from the triangle topology container
    if (!dynamic_cast<TetrahedronSetTopologyContainer*>(toModel.get()))
    {
        msg_error() << "The output topology '" << toModel.getPath() << "' is not a derived class of TetrahedronSetTopologyContainer. "
            << "Consider setting the '" << toModel.getName() << "' data attribute to a valid"
            " TetrahedronSetTopologyContainer derived object.";
        modelsOk = false;
    }
    else
    {
        TetrahedronSetTopologyModifier *to_tstm;
        toModel->getContext()->get(to_tstm);
        if (!to_tstm)
        {
            msg_error() << "No TetrahedronSetTopologyModifier found in the output topology node '"
                << toModel->getContext()->getName() << "'.";
            modelsOk = false;
        }
        else {
            m_outTopoModifier = to_tstm;
        }
    }

    if (!modelsOk)
    {
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    // INITIALISATION of Tetrahedron mesh from Hexahedron mesh :
    // Clear output topology
    toModel->clear();

    // Set the same number of points
    toModel->setNbPoints(fromModel->getNbPoints());


    sofa::helper::vector <Index>& Loc2GlobVec = *(Loc2GlobDataVec.beginEdit());

    Loc2GlobVec.clear();
    Glob2LocMap.clear();

    size_t nbcubes = fromModel->getNbHexahedra();

    // These values are only correct if the mesh is a grid topology
    int nx = 2;
    int ny = 1;
    //int nz = 1;
    {
        topology::GridTopology* grid = dynamic_cast<topology::GridTopology*>(fromModel.get());
        if (grid != nullptr)
        {
            nx = grid->getNx()-1;
            ny = grid->getNy()-1;
            //nz = grid->getNz()-1;
        }
    }

    // Tesselation of each cube into 6 tetrahedra
    for (size_t i=0; i<nbcubes; i++)
    {
        core::topology::BaseMeshTopology::Hexa c = fromModel->getHexahedron(i);
#define swap(a,b) { int t = a; a = b; b = t; }
        // TODO : swap indexes where needed (currently crash in TriangleSetContainer)
        bool swapped = false;

        if(swapping.getValue())
        {
            if (!((i%nx)&1))
            {
                // swap all points on the X edges
                swap(c[0],c[1]);
                swap(c[3],c[2]);
                swap(c[4],c[5]);
                swap(c[7],c[6]);
                swapped = !swapped;
            }
            if (((i/nx)%ny)&1)
            {
                // swap all points on the Y edges
                swap(c[0],c[3]);
                swap(c[1],c[2]);
                swap(c[4],c[7]);
                swap(c[5],c[6]);
                swapped = !swapped;
            }
            if ((i/(nx*ny))&1)
            {
                // swap all points on the Z edges
                swap(c[0],c[4]);
                swap(c[1],c[5]);
                swap(c[2],c[6]);
                swap(c[3],c[7]);
                swapped = !swapped;
            }
        }
#undef swap
        if(!swapped)
        {
            toModel->addTetra(c[0],c[5],c[1],c[6]);
            toModel->addTetra(c[0],c[1],c[3],c[6]);
            toModel->addTetra(c[1],c[3],c[6],c[2]);
            toModel->addTetra(c[6],c[3],c[0],c[7]);
            toModel->addTetra(c[6],c[7],c[0],c[5]);
            toModel->addTetra(c[7],c[5],c[4],c[0]);
        }
        else
        {
            toModel->addTetra(c[0],c[5],c[6],c[1]);
            toModel->addTetra(c[0],c[1],c[6],c[3]);
            toModel->addTetra(c[1],c[3],c[2],c[6]);
            toModel->addTetra(c[6],c[3],c[7],c[0]);
            toModel->addTetra(c[6],c[7],c[5],c[0]);
            toModel->addTetra(c[7],c[5],c[0],c[4]);
        }
        for(int j=0; j<6; j++)
            Loc2GlobVec.push_back(i);
        Glob2LocMap[i] = (unsigned int)Loc2GlobVec.size()-1;
    }

    Loc2GlobDataVec.endEdit();

    // Need to fully init the target topology
    toModel->init();

    this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}

Index Hexa2TetraTopologicalMapping::getFromIndex(Index /*ind*/)
{

    return Topology::InvalidID;
}

void Hexa2TetraTopologicalMapping::updateTopologicalMappingTopDown()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    sofa::helper::AdvancedTimer::stepBegin("Update Hexa2TetraTopologicalMapping");

    auto itBegin = fromModel->beginChange();
    auto itEnd = fromModel->endChange();

    sofa::helper::vector <Index>& Loc2GlobVec = *(Loc2GlobDataVec.beginEdit());

    while (itBegin != itEnd)
    {
        TopologyChangeType changeType = (*itBegin)->getChangeType();
        std::string topoChangeType = "Hexa2TetraTopologicalMapping - " + parseTopologyChangeTypeToString(changeType);
        sofa::helper::AdvancedTimer::stepBegin(topoChangeType);

        switch (changeType)
        {
        case core::topology::ENDING_EVENT:
        {
            m_outTopoModifier->notifyEndingEvent();
            break;
        }
        case core::topology::TETRAHEDRAREMOVED:
        {
            break;
        }
        case core::topology::TETRAHEDRAADDED:
        {
            break;
        }
        case core::topology::HEXAHEDRAREMOVED:
        {
            const auto& hexahedronArray = fromModel->getHexahedra();
            const auto& hexaIds2Remove = (static_cast<const HexahedraRemoved*>(*itBegin))->getArray();

            std::cout << "hexahedronArray: " << hexahedronArray << std::endl;
            std::cout << "hexaIds2Remove: " << hexaIds2Remove << std::endl;

            std::cout << "Loc2GlobVec: " << Loc2GlobVec << std::endl;
            std::cout << "Glob2LocMap: " << std::endl;
            for (auto ids : Glob2LocMap)
            {
                std::cout << "ids: " << ids.first << " -> " << ids.second << std::endl;
            }

            sofa::Size idLast = fromModel->getNbHexahedra() - 1;
            sofa::helper::vector< BaseMeshTopology::TetrahedronID > tetraId_to_remove;
            for (auto hexaId : hexaIds2Remove)
            {
                auto iter_1 = Glob2LocMap.find(hexaId);

                if (iter_1 == Glob2LocMap.end())
                {
                    msg_error() << " in HEXAHEDRAREMOVED process, hexa id " << hexaId << " not found in Glob2LocMap";
                    continue;
                }

                BaseMeshTopology::HexahedronID idLast = iter_1->second;
                BaseMeshTopology::HexahedronID idFirst = idLast - 5; // 6 tetra per hexa

                if (idFirst == BaseMeshTopology::InvalidID)
                {
                    msg_error() << " in HEXAHEDRAREMOVED process, tetra id " << idLast << " is not including 5 previous tetrahedron indices.";
                    continue;
                }

                for (BaseMeshTopology::HexahedronID i = idFirst; i <= idLast; i++)
                    tetraId_to_remove.push_back(i);

                Glob2LocMap.erase(Glob2LocMap.find(idLast));
                idLast--;
                Loc2GlobVec.pop_back();
            }

            // remove old triangles
            m_outTopoModifier->removeTetrahedra(tetraId_to_remove, true);
            break;
        }
        case core::topology::HEXAHEDRAADDED:
        {
            break;
        }

        }


        sofa::helper::AdvancedTimer::stepEnd(topoChangeType);
        ++itBegin;
    }

    Loc2GlobDataVec.endEdit();

    sofa::helper::AdvancedTimer::stepEnd("Update Hexa2TetraTopologicalMapping");
}


} //namespace sofa::component::topology
