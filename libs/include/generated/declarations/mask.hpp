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
 * \file /home/u172652/damon23_tvl_fpga/libs/include/generated/declarations/mask.hpp
 * \date 2023-02-27
 * \brief Mask related primitives.
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

#ifndef TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_MASK_HPP
#define TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_MASK_HPP

namespace tvl {
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "to_integral".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct to_integral{};
   }
   /*
    * \brief Forms an integral value from the most significant bits of every lane in a vector mask register.
    * \details todo.
    * \param vec_mask Vector mask register containing mask style data.
    * \return Integral value representing of the vector mask register.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::imask_type to_integral(
          const typename Vec::mask_type vec_mask
      ) {
         return functors::to_integral<Vec, Idof>::apply(
            vec_mask
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "to_vector".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct to_vector{};
   }
   /*
    * \brief Forms an vector register from an integral where all bits are set in a lane if the corresponding mask bit is set to 1.
    * \details todo.
    * \param mask Vector mask register containing mask style data.
    * \return Integral value representing the vector mask register.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::register_type to_vector(
          const typename Vec::mask_type mask
      ) {
         return functors::to_vector<Vec, Idof>::apply(
            mask
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "to_mask".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct to_mask{};
   }
   /*
    * \brief Forms a mask type from an integral.
    * \details todo.
    * \param mask Integral value representing the vector mask register.
    * \return Vector mask register containing mask style data.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::mask_type to_mask(
          const typename Vec::imask_type mask
      ) {
         return functors::to_mask<Vec, Idof>::apply(
            mask
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "imask_binary_not".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct imask_binary_not{};
   }
   /*
    * \brief Binary NOT of a vector integral mask type.
    * \details todo.
    * \param mask Mask.
    * \return todo.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::imask_type mask_binary_not(
          const typename Vec::imask_type mask
      ) {
         return functors::imask_binary_not<Vec, Idof>::apply(
            mask
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "mask_binary_and".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct mask_binary_and{};
   }
   /*
    * \brief Binary AND of two vector mask types.
    * \details todo.
    * \param first First mask.
    * \param second Second mask.
    * \return Vector mask register containing the result.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::mask_type mask_binary_and(
          const typename Vec::mask_type first, const typename Vec::mask_type second
      ) {
         return functors::mask_binary_and<Vec, Idof>::apply(
            first,second
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "imask_binary_and".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct imask_binary_and{};
   }
   /*
    * \brief Binary AND of two vector integral mask types.
    * \details todo.
    * \param first First mask.
    * \param second Second mask.
    * \return Vector mask register containing the result.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::imask_type mask_binary_and(
          const typename Vec::imask_type first, const typename Vec::imask_type second
      ) {
         return functors::imask_binary_and<Vec, Idof>::apply(
            first,second
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "mask_binary_or".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct mask_binary_or{};
   }
   /*
    * \brief Binary OR of two vector mask types.
    * \details todo.
    * \param first First mask.
    * \param second Second mask.
    * \return Vector mask register containing the result.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::mask_type mask_binary_or(
          const typename Vec::mask_type first, const typename Vec::mask_type second
      ) {
         return functors::mask_binary_or<Vec, Idof>::apply(
            first,second
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "imask_binary_or".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct imask_binary_or{};
   }
   /*
    * \brief Binary OR of two vector integral mask types.
    * \details todo.
    * \param first First mask.
    * \param second Second mask.
    * \return Vector mask register containing the result.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::imask_type mask_binary_or(
          const typename Vec::imask_type first, const typename Vec::imask_type second
      ) {
         return functors::imask_binary_or<Vec, Idof>::apply(
            first,second
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "mask_binary_xor".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct mask_binary_xor{};
   }
   /*
    * \brief Binary XOR of two vector mask types.
    * \details todo.
    * \param first First mask.
    * \param second Second mask.
    * \return Vector mask register containing the result.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::mask_type mask_binary_xor(
          const typename Vec::mask_type first, const typename Vec::mask_type second
      ) {
         return functors::mask_binary_xor<Vec, Idof>::apply(
            first,second
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "imask_binary_xor".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct imask_binary_xor{};
   }
   /*
    * \brief Binary XOR of two vector integral mask types.
    * \details todo.
    * \param first First mask.
    * \param second Second mask.
    * \return Vector mask register containing the result.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::imask_type mask_binary_xor(
          const typename Vec::imask_type first, const typename Vec::imask_type second
      ) {
         return functors::imask_binary_xor<Vec, Idof>::apply(
            first,second
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "imask_population_count".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct imask_population_count{};
   }
   /*
    * \brief todo.
    * \details todo.
    * \param mask Mask.
    * \return Numer of bits set to 1.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE unsigned int mask_population_count(
          const typename Vec::imask_type mask
      ) {
         return functors::imask_population_count<Vec, Idof>::apply(
            mask
         );
      }
   
namespace functors {
      // Forward declaration of implementation struct for TVL-primitive "integral_all_true".
      template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof>
         struct integral_all_true{};
   }
   /*
    * \brief todo.
    * \details todo.
    * \return Integral value where all values are set to 1.
    */
   template<VectorProcessingStyle Vec, ImplementationDegreeOfFreedom Idof = workaround>
      [[nodiscard]] TVL_FORCE_INLINE typename Vec::imask_type integral_all_true(
         
      ) {
         return functors::integral_all_true<Vec, Idof>::apply(
            
         );
      }
} // end of namespace tvl
#endif //TUD_D2RG_TVL_HOME_U172652_DAMON23_TVL_FPGA_LIBS_INCLUDE_GENERATED_DECLARATIONS_MASK_HPP