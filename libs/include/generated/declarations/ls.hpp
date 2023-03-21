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
 * \file /home/u172652/damon23_tvl_fpga/libs/include/generated/declarations/ls.hpp
 * \date 2023-02-27
 * \brief Load/Store primitives
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

#ifndef TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_LS_HPP
#define TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_LS_HPP

namespace tvl {
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "load".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct load{};
   }
   /*
    * \brief Loads data from aligned memory into a vector register.
    * \details todo.
    * \param memory Aligned memory which should be transferred into a vector register.
    * \return Vector containing the loaded data.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type load(
          const typename Vec::base_type * memory
      ) {
         return functors::load<Vec, Idof>::apply(
            memory
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "loadu".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct loadu{};
   }
   /*
    * \brief Loads data from (un)aligned memory into a vector register.
    * \details todo.
    * \param memory (Un)aligned memory which should be transferred into a vector register.
    * \return Vector containing the loaded data.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type loadu(
          const typename Vec::base_type * memory
      ) {
         return functors::loadu<Vec, Idof>::apply(
            memory
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "store".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct store{};
   }
   /*
    * \brief Stores data from a vector register to aligned memory.
    * \details todo.
    * \param memory Aligned memory where the data should be stored into.
    * \param data Vector containing the data.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      TVL_FORCE_INLINE void store(
          typename Vec::base_type * memory, const typename Vec::register_type data
      ) {
         functors::store<Vec, Idof>::apply(
            memory,data
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "storeu".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct storeu{};
   }
   /*
    * \brief Stores data from a vector register to (un)aligned memory.
    * \details todo.
    * \param memory (Un)aligned memory where the data should be stored into.
    * \param data Vector containing the data.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      TVL_FORCE_INLINE void storeu(
          typename Vec::base_type * memory, const typename Vec::register_type data
      ) {
         functors::storeu<Vec, Idof>::apply(
            memory,data
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "to_array".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct to_array{};
   }
   /*
    * \brief Stores SIMD register to array.
    * \details todo.
    * \param data Data register.
    * \return Array containing the values from data.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE __attribute__((__aligned__(Vec::vector_alignment()))) std::array<typename Vec::base_type, Vec::vector_element_count()> to_array(
          const typename Vec::register_type data
      ) {
         return functors::to_array<Vec, Idof>::apply(
            data
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "set1".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct set1{};
   }
   /*
    * \brief Broadcasts a single value into all lanes of a vector register.
    * \details todo.
    * \param value Value which should be broadcasted.
    * \return Vector containing the same value in all lanes.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type set1(
          const typename Vec::base_type value
      ) {
         return functors::set1<Vec, Idof>::apply(
            value
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "set_zero".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct set_zero{};
   }
   /*
    * \brief Set all lanes to zero.
    * \details todo.
    * \return Vector containing 0 in all lanes.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type set_zero(
         
      ) {
         return functors::set_zero<Vec, Idof>::apply(
            
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "set".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct set{};
   }
   /*
    * \brief Transfers provided elements into a vector register.
    * \details todo.
    * \param args Values which should be transferred.
    * \return Vector register containing the values.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround, typename... Ts>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type set(
         Ts... args
      ) {
         return functors::set<Vec, Idof>::apply(
            args...
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "sequence".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct sequence{};
   }
   /*
    * \brief Creates a sequence [0..SIMD-Reg-Element-Count].
    * \details todo.
    * \return Vector register containing the sequence.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type sequence(
         
      ) {
         return functors::sequence<Vec, Idof>::apply(
            
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "custom_sequence".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct custom_sequence{};
   }
   /*
    * \brief Creates a sequence.
    * \details todo.
    * \param start Start value for sequence.
    * \param stepwidth Stepwidth.
    * \return Vector register containing the sequence.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type custom_sequence(
          typename Vec::base_type start=0, [[maybe_unused]] typename Vec::base_type stepwidth=1
      ) {
         return functors::custom_sequence<Vec, Idof>::apply(
            start,stepwidth
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "gather".
      template<VectorProcessingStyle Vec, int N, ImplementationDegreeOfFreedom Idof>
         struct gather{};
   }
   /*
    * \brief Transfers data from arbitrary locations into a vector register.
    * \details todo.
    * \param memory (Start)pointer of the memory (which is used as base for address calculation).
    * \param index Offsets array containing simd<offset_t, EXTENSION> register_types containing relative offsets to the start pointer (the number of array elements depend on the sizeof(offset_type)/sizeof(base_type) ratio, where sizeof(offset_type) should be 8).
    * \param scale Scale.
    * \return Vector containing gathered data.
    */
   template<VectorProcessingStyle Vec, int N, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type gather(
          const void * memory, const typename Vec::offset_base_register_type index, [[maybe_unused]] std::integral_constant<int, N> scale
      ) {
         return functors::gather<Vec, N, Idof>::apply(
            memory,index,scale
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "mask_gather".
      template<VectorProcessingStyle Vec, int N, ImplementationDegreeOfFreedom Idof>
         struct mask_gather{};
   }
   /*
    * \brief Transfers data from arbitrary locations into a vector register.
    * \details todo.
    * \param mask Mask indicating which lanes should be gathered.
    * \param source Vector register containing values which should be preserved depending on the mask (if mask[i] == 0).
    * \param memory (Start)pointer of the memory (which is used as base for address calculation).
    * \param index Offsets array containing simd<offset_t, EXTENSION> register_types containing relative offsets to the start pointer (the number of array elements depend on the sizeof(offset_type)/sizeof(base_type) ratio, where sizeof(offset_type) should be 8).
    * \param scale Scale.
    * \return Vector containing gathered data.
    */
   template<VectorProcessingStyle Vec, int N, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type gather(
          const typename Vec::mask_type mask, const typename Vec::register_type source, const void * memory, const typename Vec::offset_base_register_type index, [[maybe_unused]] std::integral_constant<int, N> scale
      ) {
         return functors::mask_gather<Vec, N, Idof>::apply(
            mask,source,memory,index,scale
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "scatter".
      template<VectorProcessingStyle Vec, int N, ImplementationDegreeOfFreedom Idof>
         struct scatter{};
   }
   /*
    * \brief Transfers data from a vector register to an arbitrary locations.
    * \details todo.
    * \param data Vector register containing values which should be scattered to memory.
    * \param memory (Start)pointer of the memory (which is used as base for address calculation).
    * \param index Offsets array containing simd<offset_t, EXTENSION> register_types containing relative offsets to the start pointer (the number of array elements depend on the sizeof(offset_type)/sizeof(base_type) ratio, where sizeof(offset_type) should be 8).
    * \param scale Scale.
    */
   template<VectorProcessingStyle Vec, int N, ImplementationDegreeOfFreedom Idof = workaround>
      TVL_FORCE_INLINE void scatter(
          const typename Vec::register_type data, void * memory, const typename Vec::offset_base_register_type index, [[maybe_unused]] std::integral_constant<int, N> scale
      ) {
         functors::scatter<Vec, N, Idof>::apply(
            data,memory,index,scale
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "mask_scatter".
      template<VectorProcessingStyle Vec, int N, ImplementationDegreeOfFreedom Idof>
         struct mask_scatter{};
   }
   /*
    * \brief Transfers data from a vector register to an arbitrary locations.
    * \details todo.
    * \param mask Mask indicating which lanes should be scattered.
    * \param data Vector register containing values which should be scattered to memory.
    * \param memory (Start)pointer of the memory (which is used as base for address calculation).
    * \param index Offsets array containing simd<offset_t, EXTENSION> register_types containing relative offsets to the start pointer (the number of array elements depend on the sizeof(offset_type)/sizeof(base_type) ratio, where sizeof(offset_type) should be 8).
    * \param scale Scale.
    */
   template<VectorProcessingStyle Vec, int N, ImplementationDegreeOfFreedom Idof = workaround>
      TVL_FORCE_INLINE void scatter(
          const typename Vec::mask_type mask, const typename Vec::register_type data, void * memory, const typename Vec::offset_base_register_type index, [[maybe_unused]] std::integral_constant<int, N> scale
      ) {
         functors::mask_scatter<Vec, N, Idof>::apply(
            mask,data,memory,index,scale
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "compress_store".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct compress_store{};
   }
   /*
    * \brief Stores elements from data consecutively, if the corresponding bit in mask is set to 1.
    * \details todo.
    * \param mask Mask
    * \param memory Memory
    * \param data Data
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      TVL_FORCE_INLINE void compress_store(
          const typename Vec::imask_type mask, typename Vec::base_type * memory, const typename Vec::register_type data
      ) {
         functors::compress_store<Vec, Idof>::apply(
            mask,memory,data
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "expand_load".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct expand_load{};
   }
   /*
    * \brief Loads contiguos data from a specified memory location and puts the elements using write mask.
    * \details todo.
    * \param mask Mask
    * \param src Source register. Data is kept if corresponding mask bit is set to 0.
    * \param memory Memory
    * \return Vector containing the elements.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type expand_load(
          const typename Vec::imask_type mask, const typename Vec::register_type src, typename Vec::base_type * memory
      ) {
         return functors::expand_load<Vec, Idof>::apply(
            mask,src,memory
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "load_convert_up".
      template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof>
         struct load_convert_up{};
   }
   /*
    * \brief todo.
    * \details todo.
    * \param memory Memory
    * \return Vector containing the data.
    */
   template<VectorProcessingStyle Vec, typename ToType, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename ToType::register_type load_convert_up(
          typename Vec::base_type const * memory
      ) {
         return functors::load_convert_up<Vec, ToType, Idof>::apply(
            memory
         );
      }
} // end of namespace tvl
#endif //TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_LS_HPP