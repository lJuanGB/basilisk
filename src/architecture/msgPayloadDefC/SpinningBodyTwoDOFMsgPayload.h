/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef spinningBodyTwoDOFSimMsg_h
#define spinningBodyTwoDOFSimMsg_h


 /*! @brief Structure used to define the individual Spinning Body data message*/
typedef struct {
    double theta1;                   //!< [rad], spinning body angular displacement 1
    double theta2;                   //!< [rad], spinning body angular displacement 2
    double thetaDot1;                //!< [rad/s], spinning body angular displacement rate 1
    double thetaDot2;                //!< [rad/s], spinning body angular displacement rate 2

}SpinningBodyTwoDOFMsgPayload;


#endif /* spinningBodyTwoDOFSimMsg_h */
