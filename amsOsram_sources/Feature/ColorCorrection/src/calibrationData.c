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
 */

#include "calibrationData.h"

// use Matlab script "main_char2coef.m" to get the following table from characteristic curves
const colorCorrFunCoef_t ColorCorrNightCoef = {
.R={.XRel={.a=-1.5097635e-06f, .b=-5.7292726e-03f},.YRel={.a=+3.0904633e-06f, .b=-6.2814893e-03f},.ZRel={.a=-2.4749445e-05f, .b=-2.9072803e-03f}},
.G={.XRel={.a=-1.3369178e-06f, .b=+2.3074878e-04f},.YRel={.a=-2.0818420e-06f, .b=-1.7270739e-03f},.ZRel={.a=+9.8478663e-07f, .b=-2.3840605e-03f}},
.B={.XRel={.a=-4.3205298e-06f, .b=-2.0494016e-03f},.YRel={.a=+1.9860944e-06f, .b=+1.2126732e-03f},.ZRel={.a=-4.6195983e-06f, .b=-1.7503711e-03f}},
};

const colorCorrFunCoef_t ColorCorrDayCoef = {
.R={.XRel={.a=-1.5097635e-06f, .b=-5.7292726e-03f},.YRel={.a=+3.0904633e-06f, .b=-6.2814893e-03f},.ZRel={.a=-2.4749445e-05f, .b=-2.9072803e-03f}},
.G={.XRel={.a=-1.3369178e-06f, .b=+2.3074878e-04f},.YRel={.a=-2.0818420e-06f, .b=-1.7270739e-03f},.ZRel={.a=+9.8478663e-07f, .b=-2.3840605e-03f}},
.B={.XRel={.a=-4.3205298e-06f, .b=-2.0494016e-03f},.YRel={.a=+1.9860944e-06f, .b=+1.2126732e-03f},.ZRel={.a=-4.6195983e-06f, .b=-1.7503711e-03f}},
};

