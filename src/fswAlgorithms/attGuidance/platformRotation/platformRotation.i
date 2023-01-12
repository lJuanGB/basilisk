/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
%module platformRotation
%{
   #include "platformRotation.h"
%}

%include "swig_conly_data.i"
%constant void SelfInit_platformRotation(void*, uint64_t);
%ignore SelfInit_platformRotation;
%constant void Reset_platformRotation(void*, uint64_t, uint64_t);
%ignore Reset_platformRotation;
%constant void Update_platformRotation(void*, uint64_t, uint64_t);
%ignore Update_platformRotation;

%include "platformRotation.h"

%include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
struct VehicleConfigMsg_C;
%include "architecture/msgPayloadDefC/RWArrayConfigMsgPayload.h"
struct RWArrayConfigMsg_C;
%include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
struct RWSpeedMsg_C;
%include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
struct CmdTorqueBodyMsg_C;
%include "architecture/msgPayloadDefC/SpinningBodyMsgPayload.h"
struct SpinningBodyMsg_C;
%include "architecture/msgPayloadDefC/BodyHeadingMsgPayload.h"
struct BodyHeadingMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
