/*****************************************************************************
 * Copyright 2022 by ams OSRAM AG                                                  *
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

#ifndef DRIVER_BUFFERCONTROL_INC_BUFFERCONTROL_H_
#define DRIVER_BUFFERCONTROL_INC_BUFFERCONTROL_H_

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
  BUFFER_NO_ERROR = 0,
  BUFFER_COULD_NOT_ADD_DATA = 0xF1,
  BUFFER_FULL = 0xF2,
  BUFFER_STUCK_IN_LOOP = 0xF3,
  BUFFER_DOUBLE_WRITE = 0xF4,
  BUFFER_ERROR = 0xF5

} bufferErrorCode_t;

typedef enum
{
  BUFFER_STATUS_UNDEF, BUFFER_STATUS_OK, BUFFER_STATUS_ERROR, BUFFER_STATUS_FULL
} bufferStatus_t;


void buffer_init (void);
bufferErrorCode_t buffer_set_new_data (uint8_t *p_data, uint8_t lenght,
                                       uint8_t answerWait, uint32_t delay);
bufferErrorCode_t check_buffer (uint8_t *p_data, uint8_t lenght,
                                uint8_t answerWait, uint8_t delay,
                                uint8_t *p_activeBuffer);
bufferStatus_t get_buffer_control_status (void);
void update_buffer_control_status (void);
uint8_t get_next_buffer (bufferErrorCode_t *err);
void buffer_fill_up (uint8_t *p_data, uint8_t lenght, bool answerWait,
                     uint32_t delay);
void part_function_buffer_set_new_data (uint8_t *bufferSwitchCounter,
                                        uint8_t lenght);


#endif /* DRIVER_BUFFERCONTROL_INC_BUFFERCONTROL_H_ */
