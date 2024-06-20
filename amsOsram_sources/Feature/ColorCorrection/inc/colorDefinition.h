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

#ifndef DEMOS_COLORCORRECTION_INC_COLORDEFINITION_H_
#define DEMOS_COLORCORRECTION_INC_COLORDEFINITION_H_

#ifdef __cplusplus
extern "C"
  {
#endif

/**
 * \brief colors of LED chips
 * 
 */
enum ledColor
{
  red, green, blue
};
typedef enum ledColor ledColor_t;

// Color coordinates
/*!
 *  \struct xyY_t
 *  \brief CIE x/y and luminous intensity
 */
typedef struct
{
  float cx;  // CIE xy coordinates and luminous intensity Y
  float cy;
  float Y;
} xyY_t;

/*!
 *  \struct upvpY_t
 *  \brief  CIE 1976 UCS u'v' and luminous intensity Y
 */
typedef struct
{
  float up;  //  CIE 1976 UCS u'v' coordinates and luminous intensity Y
  float vp;
  float Y;
} upvpY_t;

/**
 *  \struct XYZ_t
 *  \brief Tristimulus values X, Y, Z
 */
typedef struct
{
  float X;
  float Y;
  float Z;
} XYZ_t;

/*!
 *  \struct RGB_XYZ_t
 *  \brief Tristimulus values for all LED chips (red, green, blue)
 *  \var RGB_XYZ_t RGB_XYZ_t::R
 *  \brief tristimulus values X, Y, Z of red chip
 *  \var RGB_XYZ_t RGB_XYZ_t::G
 *  \brief tristimulus values X, Y, Z of green chip
 *  \var RGB_XYZ_t RGB_XYZ_t::B
 *  \brief tristimulus values X, Y, Z of blue chip
 */
typedef struct
{
  XYZ_t R; // red
  XYZ_t G; // green
  XYZ_t B; // blue
} RGB_XYZ_t;

/**
 * \struct
 * \brief Tristimulus values for all LED chips (red, green, blue) and current
 *  configurations (day, night).
 *
 */
typedef struct
{
  RGB_XYZ_t day;    // driving current 50mA
  RGB_XYZ_t night;  // driving current 10mA
} DN_RGB_XYZ_t;

// Color conversion functions
/**
 * \brief convert CIE cxy -> u'v'
 *
 * \param cx CIE cx coordinate
 * \param cy CIE cy coordinate
 * \param p_up pointer to  CIE 1976 UCS u' coordinate
 * \param p_vp pointer to  CIE 1976 UCS v' coordinate
 */
void cxy2upvp (float cx, float cy, float *p_up, float *p_vp); // convert CIE cxy -> u'v'

/**
 * \brief convert  CIE 1976 UCS u'v' -> CIE cxy
 *
 * \param up  CIE 1976 UCS u' coordinate
 * \param vp  CIE 1976 UCS v' coordinate
 * \param p_cx pointer to CIE cx coordinate
 * \param p_cy pointer to CIE cy coordinate
 */
void upvp2cxy (float up, float vp, float *p_cx, float *p_cy);

/**
 * \brief convert CIE cx, cy & Y -> tristimulus X, Z
 *
 * \param cx CIE cx coordinate
 * \param cy CIE cy coordinate
 * \param Y Tristimulus value Y
 * \param p_X pointer to Tristimulus value X
 * \param p_Z to Z Tristimulus value Z
 */
void cxyY2XZ (float cx, float cy, float Y, float *p_X, float *p_Z);

/**
 * \brief convert CIE 1976 UCS u', v' & Y -> tristimulus X, Z
 *
 * \param up CIE 1976 UCS u' coordinate
 * \param vp CIE 1976 UCS v' coordinate
 * \param Y Tristimulus value Y
 * \param p_X pointer to Tristimulus value X
 * \param p_Z pointer to Tristimulus value Z
 */
void upvpY2XZ (float up, float vp, float Y, float *p_X, float *p_Z);

/**
 * \brief convert CIE cx, cy & Y -> tristimulus X,Z
 *
 * \param cx CIE cx coordinate
 * \param cy CIE cy coordinate
 * \param Y Tristimulus value Y
 * \param p_XYZ pointer to tristimulus values XYZ
 */
void cxyY2XYZ (float cx, float cy, float Y, XYZ_t *p_XYZ);

/**
 * \brief CIE 1976 UCS u', v' & Y -> tristimulus X,Y,Z
 *
 * \param up CIE 1976 UCS u' coordinate
 * \param vp CIE 1976 UCS v' coordinate
 * \param Y Tristimulus value Y
 * \param p_XYZ pointer to tristimulus values XYZ
 */
void upvpY2XYZ (float up, float vp, float Y, XYZ_t *p_XYZ);

/**
 * \brief convert tristimulus X,Y,Z -> CIE cxy
 *
 * \param X Tristimulus value Z
 * \param Y Tristimulus value Y
 * \param Z Tristimulus value Z
 * \param p_cx pointer to CIE cx coordinate
 * \param p_cy pointer to CIE cy coordinate
 */
void XYZ2cxy (float X, float Y, float Z, float *p_cx, float *p_cy);

/**
 * \brief convert tristimulus X,Y,Z ->  CIE 1976 UCS u'v' & Y
 *
 * \param X Tristimulus value Z
 * \param Y Tristimulus value Y
 * \param Z Tristimulus value Z
 * \param p_up CIE 1976 UCS u' coordinate
 * \param p_vp CIE 1976 UCS v' coordinate
 */
void XYZ2upvp (float X, float Y, float Z, float *p_up, float *p_vp);

#ifdef __cplusplus
}
#endif

#endif /* DEMOS_COLORCORRECTION_INC_COLORDEFINITION_H_ */
