/**************************************************************************
 * 
 * Copyright (c) 2024 JAKA Robotics, Ltd. All Rights Reserved.
 * 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * For support or inquiries, please contact support@jaka.com.
 * 
 * File: jkerr.h
 * @author star@jaka
 * @date Nov-30-2021 
 *  
**************************************************************************/


#ifndef _JHERR_H_
#define _JHERR_H_

#define ERR_SUCC                 0              //success
#define ERR_FUCTION_CALL_ERROR   2              //interface error or controller not support
#define ERR_INVALID_HANDLER		 -1             //invalid handler
#define	ERR_INVALID_PARAMETER    -2             //invalid parameter
#define ERR_COMMUNICATION_ERR	 -3             //fail to connect
#define ERR_KINE_INVERSE_ERR     -4             //kine_inverse error
#define ERR_EMERGENCY_PRESSED    -5             //e-stop
#define ERR_NOT_POWERED          -6             //not power on
#define ERR_NOT_ENABLED          -7             //not enable
#define ERR_DISABLE_SERVOMODE    -8             //not in servo mode
#define ERR_NOT_OFF_ENABLE       -9             //must turn off enable first
#define ERR_PROGRAM_IS_RUNNING   -10            //cannot operate, program is running
#define ERR_CANNOT_OPEN_FILE     -11            //cannot open file, or file doesn't exist
#define ERR_MOTION_ABNORMAL      -12            //motion abnormal
#define ERR_FTP_PERFORM			 -14            //ftp error
#define ERR_VALUE_OVERSIZE       -15            // socket msg or value oversize
#define ERR_KINE_FORWARD         -16             //kine_forward error
#define ERR_EMPTY_FOLDER         -17             //not support empty folder
#define ERR_INVALID_DH            -18             // wrong DH
#define ERR_INVALID_ADMIT_CTRL_CONFIG            -19             // 

#define ERR_PROTECTIVE_STOP      -20            // protective stop
#define ERR_EMERGENCY_STOP      -21            // emergency stop
#define ERR_SOFT_LIMIT          -22             // on soft limit

#define ERR_CMD_ENCODE          -30             // fail to encode cmd string
#define ERR_CMD_DECODE          -31             // fail to decode cmd string    
#define ERR_UNCOMPRESS          -32             // fail to uncompress port 10004 string     

#define ERR_MOVEL               -40             // move linear error
#define ERR_MOVEJ               -41             // move joint error
#define ERR_MOVEC               -42             // move circular error

#define ERR_MOTION_TIMEOUT          -50             // block_wait timeout
#define ERR_POWERON_TIMEOUT         -51             // power on timeout
#define ERR_POWEROFF_TIMEOUT        -52             // power off timeout
#define ERR_ENABLE_TIMEOUT          -53             // enable timeout
#define ERR_DISABLE_TIMEOUT         -54             // disable robot timeout
#define ERR_USERFRAME_SET_TIMEOUT   -55             // fail to set userframe
#define ERR_TOOL_SET_TIMEOUT        -56             // fail to set tool
#define ERR_EDG_INIT                -57         // edg init failed
#define ERR_EDG_RUNNING             -58         // edg running, customer can only use edg interface
#define ERR_EDG_MODE             -59         // The current edg mode does not support using edg_servo_j or edg_servo_p.

#define ERR_IO_SET_TIMEOUT      -60             // fail to set IO
#define ERR_OPERATION_TIMEOUT      -61             // operation timeout

#define ERR_SERVO_FULL -62  // Servo queue is full
#define ERR_SERVO_NOT_RUN -63  // Servo is not running

#define ERR_NO_IMPLEMENT -9997     // not implement
#define ERR_DEPRECATE -9998     // deprecated interface, will be removed in the future, function is still available
#define ERR_OBSOLETE -9999      // obsolete interface, will be removed in the future, function is unavailable



//note: if you got errCode which is larger than 10000, you should minus 10000 and compare with following grpc status code
// enum StatusCode {
//   /// Not an error; returned on success.
//   OK = 0,

//   /// The operation was cancelled (typically by the caller).
//   CANCELLED = 1,

//   /// Unknown error. An example of where this error may be returned is if a
//   /// Status value received from another address space belongs to an error-space
//   /// that is not known in this address space. Also errors raised by APIs that
//   /// do not return enough error information may be converted to this error.
//   UNKNOWN = 2,

//   /// Client specified an invalid argument. Note that this differs from
//   /// FAILED_PRECONDITION. INVALID_ARGUMENT indicates arguments that are
//   /// problematic regardless of the state of the system (e.g., a malformed file
//   /// name).
//   INVALID_ARGUMENT = 3,

//   /// Deadline expired before operation could complete. For operations that
//   /// change the state of the system, this error may be returned even if the
//   /// operation has completed successfully. For example, a successful response
//   /// from a server could have been delayed long enough for the deadline to
//   /// expire.
//   DEADLINE_EXCEEDED = 4,

//   /// Some requested entity (e.g., file or directory) was not found.
//   NOT_FOUND = 5,

//   /// Some entity that we attempted to create (e.g., file or directory) already
//   /// exists.
//   ALREADY_EXISTS = 6,

//   /// The caller does not have permission to execute the specified operation.
//   /// PERMISSION_DENIED must not be used for rejections caused by exhausting
//   /// some resource (use RESOURCE_EXHAUSTED instead for those errors).
//   /// PERMISSION_DENIED must not be used if the caller can not be identified
//   /// (use UNAUTHENTICATED instead for those errors).
//   PERMISSION_DENIED = 7,

//   /// The request does not have valid authentication credentials for the
//   /// operation.
//   UNAUTHENTICATED = 16,

//   /// Some resource has been exhausted, perhaps a per-user quota, or perhaps the
//   /// entire file system is out of space.
//   RESOURCE_EXHAUSTED = 8,

//   /// Operation was rejected because the system is not in a state required for
//   /// the operation's execution. For example, directory to be deleted may be
//   /// non-empty, an rmdir operation is applied to a non-directory, etc.
//   ///
//   /// A litmus test that may help a service implementor in deciding
//   /// between FAILED_PRECONDITION, ABORTED, and UNAVAILABLE:
//   ///  (a) Use UNAVAILABLE if the client can retry just the failing call.
//   ///  (b) Use ABORTED if the client should retry at a higher-level
//   ///      (e.g., restarting a read-modify-write sequence).
//   ///  (c) Use FAILED_PRECONDITION if the client should not retry until
//   ///      the system state has been explicitly fixed. E.g., if an "rmdir"
//   ///      fails because the directory is non-empty, FAILED_PRECONDITION
//   ///      should be returned since the client should not retry unless
//   ///      they have first fixed up the directory by deleting files from it.
//   ///  (d) Use FAILED_PRECONDITION if the client performs conditional
//   ///      REST Get/Update/Delete on a resource and the resource on the
//   ///      server does not match the condition. E.g., conflicting
//   ///      read-modify-write on the same resource.
//   FAILED_PRECONDITION = 9,

//   /// The operation was aborted, typically due to a concurrency issue like
//   /// sequencer check failures, transaction aborts, etc.
//   ///
//   /// See litmus test above for deciding between FAILED_PRECONDITION, ABORTED,
//   /// and UNAVAILABLE.
//   ABORTED = 10,

//   /// Operation was attempted past the valid range. E.g., seeking or reading
//   /// past end of file.
//   ///
//   /// Unlike INVALID_ARGUMENT, this error indicates a problem that may be fixed
//   /// if the system state changes. For example, a 32-bit file system will
//   /// generate INVALID_ARGUMENT if asked to read at an offset that is not in the
//   /// range [0,2^32-1], but it will generate OUT_OF_RANGE if asked to read from
//   /// an offset past the current file size.
//   ///
//   /// There is a fair bit of overlap between FAILED_PRECONDITION and
//   /// OUT_OF_RANGE. We recommend using OUT_OF_RANGE (the more specific error)
//   /// when it applies so that callers who are iterating through a space can
//   /// easily look for an OUT_OF_RANGE error to detect when they are done.
//   OUT_OF_RANGE = 11,

//   /// Operation is not implemented or not supported/enabled in this service.
//   UNIMPLEMENTED = 12,

//   /// Internal errors. Means some invariants expected by underlying System has
//   /// been broken. If you see one of these errors, Something is very broken.
//   INTERNAL = 13,

//   /// The service is currently unavailable. This is a most likely a transient
//   /// condition and may be corrected by retrying with a backoff. Note that it is
//   /// not always safe to retry non-idempotent operations.
//   ///
//   /// \warning Although data MIGHT not have been transmitted when this
//   /// status occurs, there is NOT A GUARANTEE that the server has not seen
//   /// anything. So in general it is unsafe to retry on this status code
//   /// if the call is non-idempotent.
//   ///
//   /// See litmus test above for deciding between FAILED_PRECONDITION, ABORTED,
//   /// and UNAVAILABLE.
//   UNAVAILABLE = 14,

//   /// Unrecoverable data loss or corruption.
//   DATA_LOSS = 15,

//   /// Force users to include a default branch:
//   DO_NOT_USE = -1
// };

#endif
