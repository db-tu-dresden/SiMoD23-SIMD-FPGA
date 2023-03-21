/*==========================================================================*
 * This file is part of the TVL - a template SIMD library.                  *
 *                                                                          *
 * Copyright 2023 TVL-Team, Database Research Group TU Dresden              *
 *                                                                          *
 * Licensed under the Apache License, Version 2.0 (the "License");          *
 * you may not use this file except in compliance with the License.         *
 * You may obtain a copy of the License at                                  *
 *                                                                          *
 *     http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                          *
 * Unless required by applicable law or agreed to in writing, software      *
 * distributed under the License is distributed on an "AS IS" BASIS,        *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 * See the License for the specific language governing permissions and      *
 * limitations under the License.                                           *
 *==========================================================================*/
/*
 * \file /home/u172652/damon23_tvl_fpga/libs/include/generated/declarations/calc.hpp
 * \date 2023-02-27
 * \brief This file contains arithmetic primitives.
 * \note
 * Git-Local Url : /home/u172652/damon23_tvl_fpga/TVLGen
 * Git-Remote Url: git@github.com:db-tu-dresden/TVLGen.git
 * Git-Branch    : main
 * Git-Commit    : b763fd5 (b763fd5bf1dd9646532d2a8642e95ba90fb292d2)
 * Submodule(s):
 *   Git-Local Url : primitive_data
 *   Git-Remote Url: git@github.com:db-tu-dresden/TVLPrimitiveData.git
 *   Git-Branch    : main
 *   Git-Commit    : d419aa7 (d419aa79bb0cb49806eb3f8e137c2bc44ce37ee5)
 *
 */

#ifndef TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_CALC_HPP
#define TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_CALC_HPP

namespace tvl {
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "add".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct add{};
   }
   /*
    * \brief Adds two vector registers.
    * \details todo.
    * \param vec_a First vector.
    * \param vec_b Second vector.
    * \return Vector containing result of the addition.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type add(
          const typename Vec::register_type vec_a, const typename Vec::register_type vec_b
      ) {
         return functors::add<Vec, Idof>::apply(
            vec_a,vec_b
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "mask_add".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct mask_add{};
   }
   /*
    * \brief Adds two vector registers, depending on a mask: result[*] = (m[*])? vec_a[*]+vec_b[*] : vec_a[*].
    * \details todo.
    * \param mask Vector mask register indicating which elements should be added.
    * \param vec_a First vector.
    * \param vec_b Second vector.
    * \return Vector containing result of the addition.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type add(
          const typename Vec::mask_type mask, const typename Vec::register_type vec_a, const typename Vec::register_type vec_b
      ) {
         return functors::mask_add<Vec, Idof>::apply(
            mask,vec_a,vec_b
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "mul".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct mul{};
   }
   /*
    * \brief Multiplies two vector registers.
    * \details todo.
    * \param vec_a First vector.
    * \param vec_b Second vector.
    * \return Vector containing result of the multiplication.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type mul(
          const typename Vec::register_type vec_a, const typename Vec::register_type vec_b
      ) {
         return functors::mul<Vec, Idof>::apply(
            vec_a,vec_b
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "hadd".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct hadd{};
   }
   /*
    * \brief Reduces the elements to a sum.
    * \details todo.
    * \param value Input vector.
    * \return Scalar value after adding all elements in the vector.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::base_type hadd(
          const typename Vec::register_type value
      ) {
         return functors::hadd<Vec, Idof>::apply(
            value
         );
      }
} // end of namespace tvl
#endif //TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_CALC_HPP