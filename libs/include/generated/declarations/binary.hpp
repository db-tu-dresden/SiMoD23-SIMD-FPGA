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
 * \file /home/u172652/damon23_tvl_fpga/libs/include/generated/declarations/binary.hpp
 * \date 2023-02-27
 * \brief Bit manipulation primitives.
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

#ifndef TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_BINARY_HPP
#define TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_BINARY_HPP

namespace tvl {
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "binary_and".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct binary_and{};
   }
   /*
    * \brief Binary ANDs two vector registers.
    * \details todo.
    * \param a First vector.
    * \param b Second vector.
    * \return Vector containing result of the binary AND.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type binary_and(
          const typename Vec::register_type a, const typename Vec::register_type b
      ) {
         return functors::binary_and<Vec, Idof>::apply(
            a,b
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "binary_xor".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct binary_xor{};
   }
   /*
    * \brief Binary XORs two vector registers.
    * \details todo.
    * \param a First vector.
    * \param b Second vector.
    * \return Vector containing result of the binary XOR.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type binary_xor(
          const typename Vec::register_type a, const typename Vec::register_type b
      ) {
         return functors::binary_xor<Vec, Idof>::apply(
            a,b
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "shift_right".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct shift_right{};
   }
   /*
    * \brief Shifts data to right by n bits (shifting in 0).
    * \details todo.
    * \param data Data.
    * \param shift Amount of bits, data should be shifted.
    * \return Vector containing result of the right shift.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type shift_right(
          const typename Vec::register_type data, const int shift
      ) {
         return functors::shift_right<Vec, Idof>::apply(
            data,shift
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "shift_left".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct shift_left{};
   }
   /*
    * \brief Shifts data to left by n bits (shifting in 0).
    * \details todo.
    * \param data Data.
    * \param shift Amount of bits, data should be shifted.
    * \return Vector containing result of the right shift.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type shift_left(
          const typename Vec::register_type data, const int shift
      ) {
         return functors::shift_left<Vec, Idof>::apply(
            data,shift
         );
      }
} // end of namespace tvl
#endif //TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_BINARY_HPP