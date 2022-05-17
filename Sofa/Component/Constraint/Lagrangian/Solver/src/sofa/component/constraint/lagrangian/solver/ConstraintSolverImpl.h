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
#pragma once
#include <sofa/component/constraint/lagrangian/solver/config.h>

#include <sofa/core/behavior/ConstraintSolver.h>

#include <sofa/simulation/MechanicalVisitor.h>

#include <sofa/linearalgebra/FullMatrix.h>

#include <sofa/core/ConstraintParams.h>

namespace sofa::component::constraint::lagrangian::solver
{


class SOFA_COMPONENT_CONSTRAINT_LAGRANGIAN_SOLVER_API ConstraintProblem
{
public:
    sofa::linearalgebra::LPtrFullMatrix<SReal> W;
    sofa::linearalgebra::FullVector<SReal> dFree, f;

    ConstraintProblem();
    virtual ~ConstraintProblem();

    SReal tolerance;
    int maxIterations;

    virtual void clear(int nbConstraints);
    int getDimension()	{ return dimension; }
    SReal** getW()		{ return W.lptr(); }
    SReal* getDfree()	{ return dFree.ptr(); }
    SReal* getF()		{ return f.ptr(); }

    virtual void solveTimed(SReal tolerance, int maxIt, SReal timeout) = 0;

    unsigned int getProblemId();

protected:
    int dimension;
    unsigned int problemId;
};


class SOFA_COMPONENT_CONSTRAINT_LAGRANGIAN_SOLVER_API ConstraintSolverImpl : public sofa::core::behavior::ConstraintSolver
{
public:
    SOFA_CLASS(ConstraintSolverImpl, sofa::core::behavior::ConstraintSolver);

    virtual ConstraintProblem* getConstraintProblem() = 0;

    /// Do not use the following LCPs until the next call to this function.
    /// This is used to prevent concurent access to the LCP when using a LCPForceFeedback through an haptic thread.
    virtual void lockConstraintProblem(sofa::core::objectmodel::BaseObject* from, ConstraintProblem* p1, ConstraintProblem* p2=nullptr) = 0;
};



/// Gets the vector of constraint violation values
class MechanicalGetConstraintViolationVisitor : public simulation::BaseMechanicalVisitor
{
public:

    MechanicalGetConstraintViolationVisitor(const core::ConstraintParams* params, sofa::linearalgebra::BaseVector *v)
        : simulation::BaseMechanicalVisitor(params)
        , cparams(params)
        , m_v(v)
    {}

    Result fwdConstraintSet(simulation::Node* node, core::behavior::BaseConstraintSet* c) override
    {
        ctime_t t0 = begin(node, c);
        c->getConstraintViolation(cparams, m_v);
        end(node, c, t0);
        return RESULT_CONTINUE;
    }

    /// This visitor must go through all mechanical mappings, even if isMechanical flag is disabled
    bool stopAtMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* /*map*/) override
    {
        return false; // !map->isMechanical();
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    const char* getClassName() const override { return "MechanicalGetConstraintViolationVisitor";}

private:
    /// Constraint parameters
    const sofa::core::ConstraintParams *cparams;

    /// Vector for constraint values
    sofa::linearalgebra::BaseVector* m_v;
};

} //namespace sofa::component::constraint::lagrangian::solver