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

/*
 * osire.c
 *
 *  @date 14.04.2022
 *  @autor cker
 */


#include <Hal/Osire/inc/osire.h>
#include <Hal/CY_Gpios/inc/pin.h>
#include "cy_pdl.h"

#define WITH_INTERNAL_PULL_UP 0 // 1= internal pull up , 0 = external pull up

/**
 * @fn void hal_reset_osire_start(void)
 * @brief Set CS high for SPI Slave to listen on BUS
 *
 */
void hal_reset_osire_start (void)// gpio set cs high
{
  set_ext_pull_up_invalid (); //no external pull up
}
/**
 * @fn void hal_reset_osire_end(void)
 * @brief Set CS low for SPI Slave to stop listen on BUS
 *
 */
void hal_reset_osire_end (void)// gpio set cs low
{
	Cy_SysLib_Delay(1);
	set_ext_pull_up_valid();
}
