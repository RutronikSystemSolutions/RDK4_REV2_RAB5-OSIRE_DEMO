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

//#include <Hal/CY_Gpios/inc/pin.h> //? wird eigentlich nicht benötigt
#include <SwTimer/inc/swTimer.h>

#define TIMER_100MS_TICK_NEEDED 100

static volatile bool flag100ms = false;

// Timer object used
cyhal_timer_t timer_obj;

static void isr_timer(void* callback_arg, cyhal_timer_event_t event)
{
    (void)callback_arg;
    (void)event;
    flag100ms = true;     // Set the interrupt flag and process it from the application
}


cy_rslt_t sw_Timer_100ms_init()
{
    cy_rslt_t rslt;

    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                  // Timer compare value, not used
        .period        = 1000,               // Defines the timer period
        .direction     = CYHAL_TIMER_DIR_UP, // Timer counts up
        .is_compare    = false,              // Don't use compare mode
        .is_continuous = true,               // Run the timer indefinitely
        .value         = 0                   // Initial value of counter
    };

    // Initialize the timer object. Does not use pin output ('pin' is NC) and does not use a
    // pre-configured clock source ('clk' is NULL).
    rslt = cyhal_timer_init(&timer_obj, NC, NULL);

    // Apply timer configuration such as period, count direction, run mode, etc.
    if (CY_RSLT_SUCCESS == rslt)
    {
        rslt = cyhal_timer_configure(&timer_obj, &timer_cfg);
    }

    // Set the frequency of timer to 1000 Hz --> Auflösung 1ms
    if (CY_RSLT_SUCCESS == rslt)
    {
        rslt = cyhal_timer_set_frequency(&timer_obj, 10000);
    }

    if (CY_RSLT_SUCCESS == rslt)
    {
        // Assign the ISR to execute on timer interrupt
        cyhal_timer_register_callback(&timer_obj, isr_timer, NULL);

        // Set the event on which timer interrupt occurs and enable it
        cyhal_timer_enable_event(&timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);

        // Start the timer with the configured settings
        rslt = cyhal_timer_start(&timer_obj);
    }

    return rslt;
}

bool check_flag_100ms(void)
    {
    return (flag100ms);
    }

void reset_flag_100ms(void)
    {
    flag100ms = false;
    }


