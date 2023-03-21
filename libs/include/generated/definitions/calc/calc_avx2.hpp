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
 * \file /home/u172652/damon23_tvl_fpga/libs/include/generated/definitions/calc/calc_avx2.hpp
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

#ifndef TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DEFINITIONS_CALC_CALC_AVX2_HPP
#define TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DEFINITIONS_CALC_CALC_AVX2_HPP

#include <array>
#include <cstddef>
#include "../../declarations/calc.hpp"
namespace tvl {
   namespace functors {
      /**
       * @brief: Template specialization of implementation for "add" (primitive add).
       * @details:
       * Target Extension: avx2.
       *        Data Type: float
       *  Extension Flags: ['avx']
       */
      template<ImplementationDegreeOfFreedom Idof>
         struct add<simd<float, avx2>, Idof> {
            using Vec = simd<float, avx2>;
            
            using return_type = typename Vec::register_type;
            static constexpr bool has_return_value() {
                return true;
            }
            static constexpr bool native_supported() {
               return true;
            }
            [[nodiscard]] 
            TVL_FORCE_INLINE 
            static typename Vec::register_type apply(
                const typename Vec::register_type vec_a, const typename Vec::register_type vec_b
            ) {

               return _mm256_add_ps(vec_a, vec_b);
            }
         };
   } // end of namespace functors for template specialization of add for avx2 using float.
   namespace functors {
      /**
       * @brief: Template specialization of implementation for "add" (primitive add).
       * @details:
       * Target Extension: avx2.
       *        Data Type: uint32_t
       *  Extension Flags: ['avx2']
       * @note: Signed addition.
       */
      template<ImplementationDegreeOfFreedom Idof>
         struct add<simd<uint32_t, avx2>, Idof> {
            using Vec = simd<uint32_t, avx2>;
            
            using return_type = typename Vec::register_type;
            static constexpr bool has_return_value() {
                return true;
            }
            static constexpr bool native_supported() {
               return true;
            }
            [[nodiscard]] 
            TVL_FORCE_INLINE 
            static typename Vec::register_type apply(
                const typename Vec::register_type vec_a, const typename Vec::register_type vec_b
            ) {

               return _mm256_add_epi32(vec_a, vec_b);
            }
         };
   } // end of namespace functors for template specialization of add for avx2 using uint32_t.
      namespace functors {
      /**
       * @brief: Template specialization of implementation for "hadd" (primitive hadd).
       * @details:
       * Target Extension: avx2.
       *        Data Type: float
       *  Extension Flags: ['sse', 'sse2', 'ssse3', 'avx']
       * @note: This instruction needs sse3. However, most intel cpus only provide ssse3 (which is a superset sse3).
       */
      template<ImplementationDegreeOfFreedom Idof>
         struct hadd<simd<float, avx2>, Idof> {
            using Vec = simd<float, avx2>;
            
            using return_type = typename Vec::base_type;
            static constexpr bool has_return_value() {
                return true;
            }
            static constexpr bool native_supported() {
               return false;
            }
            [[nodiscard]] TVL_NO_NATIVE_SUPPORT_WARNING
            TVL_FORCE_INLINE 
            static typename Vec::base_type apply(
                const typename Vec::register_type value
            ) {
               static_assert( !std::is_same_v< Idof, native >, "The primitive hadd is not supported by your hardware natively while it is forced by using native" );

               __m128 vlow  = _mm256_castps256_ps128(value);
               __m128 vhigh = _mm256_extractf128_ps(value, 1);
               vlow = _mm_add_ps(vlow, vhigh);
               __m128 res = _mm_hadd_ps(vlow, vlow);
               return _mm_cvtss_f32(res) + _mm_cvtss_f32(_mm_castsi128_ps(_mm_bsrli_si128(_mm_castps_si128(res),sizeof(float))));
            }
         };
   } // end of namespace functors for template specialization of hadd for avx2 using float.
      namespace functors {
      /**
       * @brief: Template specialization of implementation for "hadd" (primitive hadd).
       * @details:
       * Target Extension: avx2.
       *        Data Type: uint32_t
       *  Extension Flags: ['sse2', 'ssse3', 'avx']
       * @note: Signed Addition. This instruction needs sse3. However, most intel cpus only provide ssse3 (which is a superset sse3).
       */
      template<ImplementationDegreeOfFreedom Idof>
         struct hadd<simd<uint32_t, avx2>, Idof> {
            using Vec = simd<uint32_t, avx2>;
            
            using return_type = typename Vec::base_type;
            static constexpr bool has_return_value() {
                return true;
            }
            static constexpr bool native_supported() {
               return false;
            }
            [[nodiscard]] TVL_NO_NATIVE_SUPPORT_WARNING
            TVL_FORCE_INLINE 
            static typename Vec::base_type apply(
                const typename Vec::register_type value
            ) {
               static_assert( !std::is_same_v< Idof, native >, "The primitive hadd is not supported by your hardware natively while it is forced by using native" );

               __m128i vlow = _mm256_castsi256_si128(value);
               __m128i vhigh = _mm256_extractf128_si256(value, 1);
               vlow = _mm_add_epi32(vlow, vhigh);
               __m128i res = _mm_hadd_epi32(vlow, vlow);
               return _mm_cvtsi128_si32(res) + _mm_cvtsi128_si32(_mm_bsrli_si128(res,sizeof(uint32_t)));
            }
         };
   } // end of namespace functors for template specialization of hadd for avx2 using uint32_t.
} // end of namespace tvl
#endif //TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DEFINITIONS_CALC_CALC_AVX2_HPP