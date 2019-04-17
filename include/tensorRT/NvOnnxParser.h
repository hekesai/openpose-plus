/*
 * Copyright 1993-2018 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO LICENSEE:
 *
 * This source code and/or documentation ("Licensed Deliverables") are
 * subject to NVIDIA intellectual property rights under U.S. and
 * international Copyright laws.
 *
 * These Licensed Deliverables contained herein is PROPRIETARY and
 * CONFIDENTIAL to NVIDIA and is being provided under the terms and
 * conditions of a form of NVIDIA software license agreement by and
 * between NVIDIA and Licensee ("License Agreement") or electronically
 * accepted by Licensee.  Notwithstanding any terms or conditions to
 * the contrary in the License Agreement, reproduction or disclosure
 * of the Licensed Deliverables to any third party without the express
 * written consent of NVIDIA is prohibited.
 *
 * NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
 * LICENSE AGREEMENT, NVIDIA MAKES NO REPRESENTATION ABOUT THE
 * SUITABILITY OF THESE LICENSED DELIVERABLES FOR ANY PURPOSE.  IT IS
 * PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY OF ANY KIND.
 * NVIDIA DISCLAIMS ALL WARRANTIES WITH REGARD TO THESE LICENSED
 * DELIVERABLES, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
 * NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
 * LICENSE AGREEMENT, IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY
 * SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 * WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THESE LICENSED DELIVERABLES.
 *
 * U.S. Government End Users.  These Licensed Deliverables are a
 * "commercial item" as that term is defined at 48 C.F.R. 2.101 (OCT
 * 1995), consisting of "commercial computer software" and "commercial
 * computer software documentation" as such terms are used in 48
 * C.F.R. 12.212 (SEPT 1995) and is provided to the U.S. Government
 * only as a commercial end item.  Consistent with 48 C.F.R.12.212 and
 * 48 C.F.R. 227.7202-1 through 227.7202-4 (JUNE 1995), all
 * U.S. Government End Users acquire the Licensed Deliverables with
 * only those rights set forth herein.
 *
 * Any use of the Licensed Deliverables in individual and commercial
 * software must include, in the user documentation and internal
 * comments to the code, the above Disclaimer and U.S. Government End
 * Users Notice.
 */

#ifndef NV_ONNX_PARSER_H
#define NV_ONNX_PARSER_H

#include "NvInfer.h"
#include "NvOnnxConfig.h"

#define ONNX_REQUIRED_VERSION_MAJOR 0
#define ONNX_REQUIRED_VERSION_MINOR 1
#define ONNX_REQUIRED_VERSION_PATCH 0

//!
//! \mainpage
//!
//! This is the API documentation for the Open Neural Network Exchange (ONNX) Parser for Nvidia TensorRT Inference Engine.
//! It provides information on individual functions, classes
//! and methods. Use the index on the left to navigate the documentation.
//!
//! Please see the accompanying user guide and samples for higher-level information and general advice on using TensorRT.
//!

//!
//! \file NvOnnxParser.h
//!
//! This is the API file for ONNX Parser for Nvidia TensorRT Inference Engine.
//!

namespace nvonnxparser
{
//!
//! \brief StatusCode.
//!
//! \return Result of the command invocation; true - Success; false - Failure.
//!
typedef bool StatusCode;

//!
//! \class IONNXParser
//! \brief ONNX Parser Class.
//!
class IONNXParser
{
public:

    //!
    //! \brief Parse the ONNX Model.
    //!
    virtual StatusCode parse() = 0;

    //!
    //! \brief Parse the ONNX Model.
    //!
    //! \param onnx_filename Onnx Model Filename.
    //!
    //! \param dataType The data type.
    //!
    //! \see nvinfer1::DataType
    //!
    virtual StatusCode parse(const char* onnx_filename,
                             nvinfer1::DataType dataType)
        = 0;

    virtual StatusCode reportParsingInfo() = 0;          //!< Report parsing info.
    virtual StatusCode convertToTRTNetwork() = 0;        //!< Convert ONNX model to TensorRT network.
    virtual StatusCode writeText() const = 0;            //!< Write ASCII Equivalent of ONNX model protbuf file.
    virtual StatusCode writeTextWithWeights() const = 0; //!< Write ASCII Equivalent of ONNX model protbuf file includeing the weights.

    //!
    //! \brief Returns the TRT Network.
    //!
    //! \return Returns the object corresponding to the TRT Network.
    //!
    virtual nvinfer1::INetworkDefinition* getTRTNetwork() const = 0;

    //!
    //! \brief Returns the Logger.
    //!
    //! \return Returns the Logger.
    //!
    virtual nvinfer1::ILogger* getLogger() = 0;
    virtual void destroy() = 0;

protected:
    virtual ~IONNXParser() {}
};

TENSORRTAPI IONNXParser* createONNXParser(const IOnnxConfig&);

} // namespace

#endif
