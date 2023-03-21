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
 * \file /home/u172652/damon23_tvl_fpga/libs/include/generated/declarations/convert.hpp
 * \date 2023-02-27
 * \brief Conversion primitives.
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

#ifndef TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_CONVERT_HPP
#define TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_CONVERT_HPP

namespace tvl {
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "reinterpret".
      template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof>
         struct reinterpret{};
   }
   /*
    * \brief todo.
    * \details todo.
    * \param data todo.
    * \return Casted register.
    */
   template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename ToType::register_type reinterpret(
          const typename Vec::register_type data
      ) {
         return functors::reinterpret<Vec, ToType, Idof>::apply(
            data
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "cast".
      template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof>
         struct cast{};
   }
   /*
    * \brief todo.
    * \details todo.
    * \param data todo.
    * \return Casted SIMD register.
    */
   template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename ToType::register_type cast(
          const typename Vec::register_type data
      ) {
         return functors::cast<Vec, ToType, Idof>::apply(
            data
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "convert_up".
      template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof>
         struct convert_up{};
   }
   /*
    * \brief todo.
    * \details todo.
    * \param data todo.
    * \return Array containing the upconverted simd registers.
    */
   template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE std::array<typename ToType::register_type, sizeof(typename ToType::base_type)/sizeof(typename Vec::base_type)> convert_up(
          const typename Vec::register_type data
      ) {
         return functors::convert_up<Vec, ToType, Idof>::apply(
            data
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "convert_down".
      template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof>
         struct convert_down{};
   }
   /*
    * \brief todo.
    * \details todo.
    * \param data todo.
    * \return SIMD register containing the down-converted data.
    */
   template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename ToType::register_type convert_down(
          std::array<typename Vec::register_type, sizeof(typename Vec::base_type)/sizeof(typename ToType::base_type)> data
      ) {
         return functors::convert_down<Vec, ToType, Idof>::apply(
            data
         );
      }
} // end of namespace tvl
#endif //TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_CONVERT_HPP