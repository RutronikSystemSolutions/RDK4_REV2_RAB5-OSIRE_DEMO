/*****************************************************************************
 * Copyright 2022 by ams OSRAM AG                                            *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************/
#ifndef OSP_INC_OSPCMDBUFFER_H_
#define OSP_INC_OSPCMDBUFFER_H_

#include <stdint.h>
#include <stdbool.h>
#include <Osp/inc/genericDevice.h>


/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/*****************************************************************************/
typedef struct ospCmd_t
{
  uint16_t inDeviceAddress; /**< INPUT: device address*/
  uint8_t inCmdId; /**< INPUT: OSP command identifier*/
  void *p_inParameter; /**< INPUT: pointer to parameter structure*/
  uint8_t *p_outCmdBuffer; /**< OUTPUT: buffer with requested OSP sequence*/
  uint8_t outCmdBufferLength; /**< OUTPUT: length of requested OSP sequence*/
  uint8_t outResponseLength; /**< OUTPUT: length of the expected response*/
  bool outResponseMsg; /**< OUTPUT: true if a response id expected*/
} ospCmdBuffer_t; /**< */

/*****************************************************************************/
/*****************************************************************************/

enum OSP_ERROR_CODE osp_cmd_buffer (ospCmdBuffer_t *p_cmdInfo);


#endif /* OSP_INC_OSPCMDBUFFER_H_ */
