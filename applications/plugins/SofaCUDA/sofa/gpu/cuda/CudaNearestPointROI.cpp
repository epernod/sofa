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
#include "CudaTypes.h"
#include <sofa/core/ObjectFactory.h>
#include <SofaGeneralEngine/NearestPointROI.inl>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace engine
{
template class SOFA_GPU_CUDA_API NearestPointROI<gpu::cuda::CudaVec2fTypes>;
template class SOFA_GPU_CUDA_API NearestPointROI<gpu::cuda::CudaVec3fTypes>;
template class SOFA_GPU_CUDA_API NearestPointROI<gpu::cuda::CudaVec3f1Types>;
template class SOFA_GPU_CUDA_API NearestPointROI<gpu::cuda::CudaRigid3fTypes>;
#ifdef SOFA_GPU_CUDA_DOUBLE
template class SOFA_GPU_CUDA_API NearestPointROI<gpu::cuda::CudaVec2dTypes>;
template class SOFA_GPU_CUDA_API NearestPointROI<gpu::cuda::CudaVec3dTypes>;
template class SOFA_GPU_CUDA_API NearestPointROI<gpu::cuda::CudaVec3d1Types>;
#endif // SOFA_GPU_CUDA_DOUBLE

} // namespace engine

} // namespace component

namespace gpu
{

namespace cuda
{

int NearestPointROICudaClass = core::RegisterObject("Supports GPU-side computations using CUDA")
        .add< component::engine::NearestPointROI<CudaVec2fTypes> >()
        .add< component::engine::NearestPointROI<CudaVec3fTypes> >()
        .add< component::engine::NearestPointROI<CudaVec3f1Types> >()
        .add< component::engine::NearestPointROI<CudaRigid3fTypes> >()
#ifdef SOFA_GPU_CUDA_DOUBLE
        .add< component::engine::NearestPointROI<CudaVec2dTypes> >()
        .add< component::engine::NearestPointROI<CudaVec3dTypes> >()
        .add< component::engine::NearestPointROI<CudaVec3d1Types> >()
        .add< component::engine::NearestPointROI<CudaRigid3dTypes> >()
#endif // SOFA_GPU_CUDA_DOUBLE
        ;

} // namespace cuda

} // namespace gpu

} // namespace sofa