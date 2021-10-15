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
#include <sofa/defaulttype/VecTypes.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaMiscFem/FastTetrahedralCorotationalForceField.h>
#include <SofaBaseTopology/EdgeSetTopologyModifier.h>
#include <SofaBaseTopology/TopologyData.inl>

#include <SofaSimulationGraph/SimpleApi.h>
#include <SofaSimulationGraph/DAGSimulation.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/Node.h>
using sofa::simulation::Node;

#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest;

#include <string>
using std::string;

#include <sofa/helper/system/thread/CTime.h>
#include <limits>


namespace sofa
{
using namespace sofa::defaulttype;
using namespace sofa::simpleapi;
using sofa::component::container::MechanicalObject;
using sofa::helper::system::thread::ctime_t;

template <class DataTypes>
class FastTetrahedralCorotationalForceField_test : public BaseTest
{
public:
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef MechanicalObject<DataTypes> MState;
    using FTetraFEM = sofa::component::forcefield::FastTetrahedralCorotationalForceField<DataTypes>;
    using TetraModifier = sofa::component::topology::TetrahedronSetTopologyModifier;
    using TetraContainer = sofa::component::topology::TetrahedronSetTopologyContainer;
    typedef typename FTetraFEM::TetrahedronRestInformation TetrahedronRestInformation;
    typedef typename type::vector<TetrahedronRestInformation> VecTetraInfo;
    using Vec3 = type::Vec<3, Real>;
    using Mat33 = type::Mat<3, 3, Real>;

protected:
    simulation::Simulation* m_simulation = nullptr;
    simulation::Node::SPtr m_root;
    
public:

    void SetUp() override
    {
        sofa::simpleapi::importPlugin("SofaComponentAll");
        simulation::setSimulation(m_simulation = new simulation::graph::DAGSimulation());
    }

    void TearDown() override
    {
        if (m_root != nullptr)
            simulation::getSimulation()->unload(m_root);
    }

    void createSingleTetrahedronFEMScene(Real youngModulus, Real poissonRatio, std::string method)
    {
        m_root = sofa::simpleapi::createRootNode(m_simulation, "root");
        m_root->setGravity(type::Vec3(0.0, -1.0, 0.0));
        m_root->setDt(0.01);

        createObject(m_root, "DefaultAnimationLoop");
        createObject(m_root, "DefaultVisualManagerLoop");

        createObject(m_root, "EulerImplicitSolver");
        createObject(m_root, "CGLinearSolver", { { "iterations", "20" }, { "threshold", "1e-8" } });
        createObject(m_root, "MechanicalObject", {{"template","Vec3d"}, {"position", "0 0 0  2 0 0  0 0 2  0 2 0"} });
        createObject(m_root, "TetrahedronSetTopologyContainer", { {"tetrahedra","0 1 2 3"} });
        createObject(m_root, "TetrahedronSetTopologyModifier");
        createObject(m_root, "TetrahedronSetGeometryAlgorithms", { {"template","Vec3d"} });

        createObject(m_root, "FastTetrahedralCorotationalForceField", { {"Name","FEM"}, {"template", "Vec3d"}, 
            {"youngModulus", str(youngModulus)}, {"poissonRatio", str(poissonRatio)}, {"method", method} });
        createObject(m_root, "DiagonalMass", { {"name","mass"}, {"massDensity","1.0"} });

        /// Init simulation
        sofa::simulation::getSimulation()->init(m_root.get());
    }


    void createGridFEMScene(Vec3 grid, Real youngModulus, Real poissonRatio, std::string method)
    {
        m_root = sofa::simpleapi::createRootNode(m_simulation, "root");
        m_root->setGravity(type::Vec3(0.0, 1.0, -9));
        m_root->setDt(0.01);

        createObject(m_root, "RegularGridTopology", { {"name", "grid"},
            {"n", str(grid)}, {"min", "0 0 20"}, {"max", "10 40 30"} });

        // Create BeamNode to create a tetrahedron mesh using topologyMapping
        Node::SPtr BeamNode = sofa::simpleapi::createChild(m_root, "Beam");
        createObject(BeamNode, "MechanicalObject", { {"name","Volume"}, {"template","Vec3d"}, {"position", "@../grid.position"} });
        createObject(BeamNode, "TetrahedronSetTopologyContainer", { {"name","Container"}, {"src","@../grid"} });
        createObject(BeamNode, "TetrahedronSetTopologyModifier", { {"name","Modifier"} });
        createObject(BeamNode, "TetrahedronSetGeometryAlgorithms", { {"name","GeomAlgo"}, {"template","Vec3d"} });
        createObject(BeamNode, "Hexa2TetraTopologicalMapping", { {"name","topoMap"}, {"input","@../grid"}, {"output","@Container"} });

        Node::SPtr FEMNode = sofa::simpleapi::createChild(m_root, "FEM");
        createObject(FEMNode, "EulerImplicitSolver");
        createObject(FEMNode, "CGLinearSolver", { { "iterations", "20" }, { "threshold", "1e-8" } });
        createObject(FEMNode, "MechanicalObject", { {"name","Volume"}, {"template","Vec3d"}, {"position", "@../grid.position"} });
        createObject(FEMNode, "TetrahedronSetTopologyContainer", { {"name","Container"}, {"src","@../Beam/Container"} });
        createObject(FEMNode, "TetrahedronSetTopologyModifier", { {"name","Modifier"} });
        createObject(FEMNode, "TetrahedronSetGeometryAlgorithms", { {"name","GeomAlgo"}, {"template","Vec3d"} });

        createObject(FEMNode, "BoxROI", { {"name","ROI1"}, {"box", "-1 -1 0 11 1 50"} });
        createObject(FEMNode, "FixedConstraint", { {"indices","@ROI1.indices"} });

        createObject(FEMNode, "FastTetrahedralCorotationalForceField", { {"Name","FEMTetra"}, {"template", "Vec3d"},
            {"youngModulus", str(youngModulus)}, {"poissonRatio", str(poissonRatio)}, {"method", method} });
        createObject(FEMNode, "DiagonalMass", { {"name","mass"}, {"massDensity","1.0"} });

        // Init simulation
        sofa::simulation::getSimulation()->init(m_root.get());
    }


    void checkCreation()
    {
        createSingleTetrahedronFEMScene(100, 0.45, "large");

        typename MState::SPtr dofs = m_root->getTreeObject<MState>();
        ASSERT_TRUE(dofs.get() != nullptr);
        ASSERT_EQ(dofs->getSize(), 4);

        typename FTetraFEM::SPtr FTFEM = m_root->getTreeObject<FTetraFEM>();
        ASSERT_TRUE(FTFEM.get() != nullptr);
        ASSERT_FLOAT_EQ(FTFEM->f_poissonRatio.getValue(), 0.45);
        ASSERT_FLOAT_EQ(FTFEM->f_youngModulus.getValue(), 100);
        ASSERT_EQ(FTFEM->f_method.getValue(), "large");
    }


    void checkNoMechanicalObject()
    {
        EXPECT_MSG_EMIT(Error);
        
        m_root = sofa::simpleapi::createRootNode(m_simulation, "root");
        createObject(m_root, "FastTetrahedralCorotationalForceField", { {"Name","FEM"}, {"template", "Vec3d"} });
        
        sofa::simulation::getSimulation()->init(m_root.get());
    }


    void checkNoTopology()
    {
        EXPECT_MSG_EMIT(Error);

        m_root = sofa::simpleapi::createRootNode(m_simulation, "root");
        createObject(m_root, "MechanicalObject", { {"template","Vec3d"}, {"position", "1 0 0  0 1 0  0 0 0  0 0 1"} });
        createObject(m_root, "FastTetrahedralCorotationalForceField", { {"Name","FEM"}, {"template", "Vec3d"} });

        sofa::simulation::getSimulation()->init(m_root.get());
    }


    void checkEmptyTopology()
    {
        m_root = sofa::simpleapi::createRootNode(m_simulation, "root");
        createObject(m_root, "MechanicalObject", { {"template","Vec3d"}, {"position", "1 0 0  0 1 0  0 0 0  0 0 1"} });
        createObject(m_root, "TetrahedronSetTopologyContainer");
        createObject(m_root, "TetrahedronSetTopologyModifier");
        createObject(m_root, "TetrahedronSetGeometryAlgorithms", { {"template","Vec3d"} });
        createObject(m_root, "FastTetrahedralCorotationalForceField", { {"Name","FEM"}, {"template", "Vec3d"} });

        EXPECT_MSG_EMIT(Warning);

        /// Init simulation
        sofa::simulation::getSimulation()->init(m_root.get());
    }


    void checkDefaultAttributes()
    {
        m_root = sofa::simpleapi::createRootNode(m_simulation, "root");

        createObject(m_root, "MechanicalObject", { {"template","Vec3d"}, {"position", "1 0 0  0 1 0  0 0 0  0 0 1"} });
        createObject(m_root, "TetrahedronSetTopologyContainer", { {"tetrahedra","0 1 2 3"} });
        createObject(m_root, "FastTetrahedralCorotationalForceField", { {"Name","FEM"}, {"template", "Vec3d"} });

        typename FTetraFEM::SPtr FTFEM = m_root->getTreeObject<FTetraFEM>();
        ASSERT_TRUE(FTFEM.get() != nullptr);
        ASSERT_FLOAT_EQ(FTFEM->f_poissonRatio.getValue(), 0.3);
        ASSERT_FLOAT_EQ(FTFEM->f_youngModulus.getValue(), 1000);
        ASSERT_EQ(FTFEM->f_method.getValue(), "qr");
    }


    void checkInit()
    {
        createSingleTetrahedronFEMScene(1000, 0.45, "large");

        typename FTetraFEM::SPtr FTFEM = m_root->getTreeObject<FTetraFEM>();
        ASSERT_TRUE(FTFEM.get() != nullptr);
        const sofa::type::vector<Mat33>& pInfos = FTFEM->pointInfo.getValue();
        const sofa::type::vector<Mat33>& eInfos = FTFEM->edgeInfo.getValue();
        const VecTetraInfo& tInfos = FTFEM->tetrahedronInfo.getValue();

        // Check Data vector sizes and values
        ASSERT_EQ(pInfos.size(), 4);
        ASSERT_EQ(eInfos.size(), 6);
        ASSERT_EQ(tInfos.size(), 1);
        
        const TetrahedronRestInformation& tInfo = tInfos[0];
        Mat33 zeroMat;
        Mat33 restRot = Mat33(Vec3(1, 0, 0), Vec3(0, 0, 1), Vec3(0, -1, 0));
        
        const Mat33& pInfoMat = pInfos[0];
        const Mat33& eInfoMat = eInfos[0];
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                ASSERT_FLOAT_EQ(pInfoMat[i][j], zeroMat[i][j]);
                ASSERT_FLOAT_EQ(eInfoMat[i][j], zeroMat[i][j]);
                ASSERT_FLOAT_EQ(tInfo.rotation[i][j], zeroMat[i][j]);
                ASSERT_FLOAT_EQ(tInfo.restRotation[i][j], restRot[i][j]);
            }
        }

        // Check TetrahedronRestInformation values
        ASSERT_FLOAT_EQ(tInfo.restVolume, Real(2*2*2)/6); // width * depth * height / 6
        Vec3 shapeVec0 = Vec3(-0.5, -0.5, -0.5);
        Vec3 shapeVec1 = Vec3(0.5, 0, 0);
        Vec3 shapeVec2 = Vec3(0, 0, 0.5);
        Vec3 shapeVec3 = Vec3(0, 0.5, 0);
        for (int i = 0; i < 3; ++i)
        {
            ASSERT_FLOAT_EQ(shapeVec0[i], tInfo.shapeVector[0][i]);
            ASSERT_FLOAT_EQ(shapeVec1[i], tInfo.shapeVector[1][i]);
            ASSERT_FLOAT_EQ(shapeVec2[i], tInfo.shapeVector[2][i]);
            ASSERT_FLOAT_EQ(shapeVec3[i], tInfo.shapeVector[3][i]);
        }
    }


    void checkGridInit()
    {
        Vec3 nGrid = Vec3(8, 32, 8);
        createGridFEMScene(nGrid, 4000, 0.3, "large");

        typename FTetraFEM::SPtr FTFEM = m_root->getTreeObject<FTetraFEM>();
        ASSERT_TRUE(FTFEM.get() != nullptr);
        const sofa::type::vector<Mat33>& pInfos = FTFEM->pointInfo.getValue();
        const sofa::type::vector<Mat33>& eInfos = FTFEM->edgeInfo.getValue();
        const VecTetraInfo& tInfos = FTFEM->tetrahedronInfo.getValue();

        typename TetraContainer::SPtr tetra = m_root->getTreeObject<TetraContainer>();
        ASSERT_TRUE(tetra.get() != nullptr);

        // Check Data vector sizes and values
        ASSERT_EQ(pInfos.size(), 2048);
        ASSERT_EQ(eInfos.size(), 12127);
        ASSERT_EQ(tInfos.size(), 9114);

        typename MState::SPtr dofs = FTFEM.get()->getContext()->get<MState>();
        ASSERT_TRUE(dofs.get() != nullptr);
        ASSERT_EQ(dofs->getSize(), 2048);
    }


    void checkFEMValues()
    {
        int nbrStep = 100;
        Vec3 nGrid = Vec3(8, 32, 8);
        createGridFEMScene(nGrid, 4000, 0.3, "large");

        typename FTetraFEM::SPtr FTFEM = m_root->getTreeObject<FTetraFEM>();
        ASSERT_TRUE(FTFEM.get() != nullptr);
        const sofa::type::vector<Mat33>& pInfos = FTFEM->pointInfo.getValue();
        const sofa::type::vector<Mat33>& eInfos = FTFEM->edgeInfo.getValue();
        const VecTetraInfo& tInfos = FTFEM->tetrahedronInfo.getValue();

        typename MState::SPtr dofs = FTFEM.get()->getContext()->get<MState>();
        ASSERT_TRUE(dofs.get() != nullptr);
        
        // Access dofs
        const VecCoord& positions = dofs->x.getValue();
        ASSERT_EQ(positions.size(), 2048);

        // Get last point of the beam extremity
        Index idLast = 8 * 32 - 1;
        Index idEdge = idLast * 4;
        Index idTetra = idLast * 2;

        EXPECT_NEAR(positions[idLast][0], 10, 1e-4);
        EXPECT_NEAR(positions[idLast][1], 40, 1e-4);
        EXPECT_NEAR(positions[idLast][2], 20, 1e-4);

        const TetrahedronRestInformation& tInfo = tInfos[idTetra];
        const Mat33& pInfoMat = pInfos[idLast];
        const Mat33& eInfoMat = eInfos[idEdge];

        Mat33 zeroMat;        
        Mat33 restRot = Mat33(Vec3(0.707107, 0, 0.707107), Vec3(0.707107, 0, -0.707107), Vec3(0, 1, 0));

        // check before simulation
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                ASSERT_FLOAT_EQ(pInfoMat[i][j], zeroMat[i][j]);
                ASSERT_FLOAT_EQ(eInfoMat[i][j], zeroMat[i][j]);
                ASSERT_FLOAT_EQ(tInfo.rotation[i][j], zeroMat[i][j]);
                ASSERT_FLOAT_EQ(tInfo.restRotation[i][j], restRot[i][j]);
            }
        }

        // Check TetrahedronRestInformation values
        EXPECT_NEAR(tInfo.restVolume, -0.43889, 1e-4);
        Vec3 shapeVec0 = Vec3(-0.7, 0, 0);
        Vec3 shapeVec1 = Vec3(0, -0.775, 0.7);
        Vec3 shapeVec2 = Vec3(0.7, 0, -0.7);
        Vec3 shapeVec3 = Vec3(0, 0.775, 0);
        for (int i = 0; i < 3; ++i)
        {
            ASSERT_FLOAT_EQ(shapeVec0[i], tInfo.shapeVector[0][i]);
            ASSERT_FLOAT_EQ(shapeVec1[i], tInfo.shapeVector[1][i]);
            ASSERT_FLOAT_EQ(shapeVec2[i], tInfo.shapeVector[2][i]);
            ASSERT_FLOAT_EQ(shapeVec3[i], tInfo.shapeVector[3][i]);
        }

        for (int i = 0; i < nbrStep; i++)
        {
            m_simulation->animate(m_root.get(), 0.01);
        }

        EXPECT_NEAR(positions[idLast][0], 9.98716, 1e-4);
        EXPECT_NEAR(positions[idLast][1], 40.37819, 1e-4);
        EXPECT_NEAR(positions[idLast][2], 15.43644, 1e-4);

        // new tetra rotation
        Mat33 restRot2 = Mat33(Vec3(0.999974, -0.00100892, 0.00709537), Vec3(0.00224179, 0.984393, -0.175968), Vec3(-0.00680709, 0.175979, 0.98437));
        Mat33 einfo = Mat33(Vec3(-1593.856984, -896.4065913, 509.9103837), Vec3(-869.9279654, -3904.171882, 1900.13097), Vec3(681.1196466, 2267.572097, -3013.179587));

        // check after simulation
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                ASSERT_FLOAT_EQ(pInfoMat[i][j], zeroMat[i][j]);
                EXPECT_NEAR(eInfoMat[i][j], einfo[i][j], 1e-4);
                EXPECT_NEAR(tInfo.rotation[i][j], restRot2[i][j], 1e-4);
                EXPECT_NEAR(tInfo.restRotation[i][j], restRot[i][j], 1e-4);
            }
        }

        // Check TetrahedronRestInformation values that should not have changed
        EXPECT_NEAR(tInfo.restVolume, -0.43889, 1e-4);
        for (int i = 0; i < 3; ++i)
        {
            ASSERT_FLOAT_EQ(shapeVec0[i], tInfo.shapeVector[0][i]);
            ASSERT_FLOAT_EQ(shapeVec1[i], tInfo.shapeVector[1][i]);
            ASSERT_FLOAT_EQ(shapeVec2[i], tInfo.shapeVector[2][i]);
            ASSERT_FLOAT_EQ(shapeVec3[i], tInfo.shapeVector[3][i]);
        }
    }


    void checkTopologyChanges()
    {
    //    createSimpleBeam(0.05, 20000000, 0.49);

    //    typename EdgeModifier::SPtr edgeModif = m_root->getTreeObject<EdgeModifier>();
    //    ASSERT_TRUE(edgeModif.get() != nullptr);

    //    typename BeamFEM::SPtr bFEM = m_root->getTreeObject<BeamFEM>();
    //    const VecBeamInfo& EdgeInfos = bFEM->m_beamsData.getValue();

    //    ASSERT_EQ(EdgeInfos.size(), 3);
    //    
    //    sofa::topology::SetIndex indices = { 0 };
    //    edgeModif->removeEdges(indices, true);

    //    m_simulation->animate(m_root.get(), 0.01);
    //    ASSERT_EQ(EdgeInfos.size(), 2);
    }

    void testFEMPerformance()
    {
        int nbrStep = 1000;
        int nbrTest = 10;
        Vec3 nGrid = Vec3(8, 32, 8);

        createGridFEMScene(nGrid, 4000, 0.3, "large");

        double diffTimeMs = 0;
        double timeMin = std::numeric_limits<double>::max();
        double timeMax = std::numeric_limits<double>::min();

        for (int i = 0; i < nbrTest; ++i)
        {
            ctime_t startTime = sofa::helper::system::thread::CTime::getRefTime();
            for (int i = 0; i < nbrStep; i++)
            {
                m_simulation->animate(m_root.get(), 0.01);
            }

            ctime_t diffTime = sofa::helper::system::thread::CTime::getRefTime() - startTime;
            double diffTimed = sofa::helper::system::thread::CTime::toSecond(diffTime);

            if (timeMin > diffTimed)
                timeMin = diffTimed;
            if (timeMax < diffTimed)
                timeMax = diffTimed;

            diffTimeMs += diffTimed;
            m_simulation->reset(m_root.get());
        }

        std::cout << "timeMean: " << diffTimeMs/nbrTest << std::endl;
        std::cout << "timeMin: " << timeMin << std::endl;
        std::cout << "timeMax: " << timeMax << std::endl;

        //timeMean: 11.8484
        //timeMin : 10.7996
        //timeMax : 13.5681

        //timeMean: 12.0717
        //timeMin : 11.9742
        //timeMax : 12.2916

        //timeMean: 12.9459
        //timeMin : 12.7815
        //timeMax : 13.995

        //timeMean: 12.2645
        //timeMin : 11.2855
        //timeMax : 14.7162
    }
};


typedef FastTetrahedralCorotationalForceField_test<Vec3Types> FastTetrahedralCorotationalForceField3_test;

TEST_F(FastTetrahedralCorotationalForceField3_test, checkForceField_Creation)
{
    this->checkCreation();
}

TEST_F(FastTetrahedralCorotationalForceField3_test, checkForceField_noMechanicalObject)
{
    this->checkNoMechanicalObject();
}

TEST_F(FastTetrahedralCorotationalForceField3_test, checkForceField_noTopology)
{
    this->checkNoTopology();
}

TEST_F(FastTetrahedralCorotationalForceField3_test, checkForceField_emptyTopology)
{
    this->checkEmptyTopology();
}

TEST_F(FastTetrahedralCorotationalForceField3_test, checkForceField_defaultAttributes)
{
    this->checkDefaultAttributes();
}

//// checkWrongAttributes is missing

TEST_F(FastTetrahedralCorotationalForceField3_test, checkForceField_init)
{
    this->checkInit();
}

TEST_F(FastTetrahedralCorotationalForceField3_test, checkForceField_gridInit)
{
    this->checkGridInit();
}

TEST_F(FastTetrahedralCorotationalForceField3_test, checkForceField_values)
{
    this->checkFEMValues();
}

TEST_F(FastTetrahedralCorotationalForceField3_test, checkForceField_TopologyChanges)
{
    this->checkTopologyChanges();
}

TEST_F(FastTetrahedralCorotationalForceField3_test, DISABLED_testFEMForceField_Performance)
{
    this->testFEMPerformance();
}

} // namespace sofa
