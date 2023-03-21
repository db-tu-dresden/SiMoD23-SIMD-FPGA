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
 * \file /home/u172652/damon23_tvl_fpga/libs/include/static/simd/simd_type_concepts.hpp
 * \date 2023-02-27
 * \brief TODO.
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

#ifndef TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_STATIC_SIMD_SIMD_TYPE_CONCEPTS_HPP
#define TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_STATIC_SIMD_SIMD_TYPE_CONCEPTS_HPP

#include <type_traits>
#include "../utils/type_concepts.hpp"
namespace tvl {
   
#ifdef TVL_USE_CONCEPTS
   template<typename T, typename U>
   concept TargetExtension = TVLArithmetic<U> && requires {
      typename T:: template types<U>;
      typename T:: template types<U>::default_size_in_bits;
      typename T:: template types<U>::register_t;
      typename T:: template types<U>::mask_t;
   };
   template<typename T>
   concept VectorProcessingStyle = requires {
      typename T::base_type;
      typename T::target_extension;
      typename T::register_type;
      typename T::mask_type;
      typename T::imask_type;
      typename T::offset_base_type;
      typename T::offset_base_register_type;
      //typename T::offset_register_type;
      { T::is_register_type_pointer() } -> std::same_as<bool>;
      { T::vector_size_b() } -> std::same_as<std::size_t>;
      { T::vector_size_B() } -> std::same_as<std::size_t>;
      { T::vector_element_count() } -> std::same_as<std::size_t>;
      { T::vector_alignment() } -> std::same_as<std::size_t>;
      { T::vector_mask_ratio() } -> std::same_as<std::size_t>;
      { T::mask_shift() } -> std::same_as<std::size_t>;
   } &&
      TVLArithmetic<typename T::base_type> &&
      TargetExtension<typename T::target_extension, typename T::base_type> &&
      ( T::vector_size_b() > 0 ) &&
      ( T::vector_size_B() > 0 ) &&
      ( T::vector_element_count() > 0 ) &&
      ( T::vector_alignment() > 0 ) &&
      ( T::vector_mask_ratio() > 0 ) &&
      ( T::mask_shift() > 0 );
#else
#define VectorProcessingStyle class
#endif
   struct native{};
   struct workaround{};
#ifdef TVL_USE_CONCEPTS
   template<class T>
   concept ImplementationDegreeOfFreedom = std::is_same_v<T, native> || std::is_same_v<T, workaround>;
#else
#define ImplementationDegreeOfFreedom class
#endif
} // end of namespace tvl
#endif //TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_STATIC_SIMD_SIMD_TYPE_CONCEPTS_HPP