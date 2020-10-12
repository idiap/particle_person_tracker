// Copyright (c) 2011-2020 Idiap Research Institute
//
// cvpCalcOpticalFlowPyrLK - method to extract sparse 2D motion vectors using
//                           the KLT algorithm
//
// Authors: Rémi Emonet (remi.emonet@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <cv.h>
#include <cvaux.h>
#include <float.h>
#include <stdio.h>

// LOCAL INCLUDES
#include "opencvplus/IdiapCVOpticalFlow.h"              // declaration of this

///////////////////////////////


#define  ICV_DEF_GET_RECT_SUB_PIX_FUNC( flavor, srctype, dsttype, worktype, \
                                        cast_macro, scale_macro, cast_macro2 )\
CvStatus CV_STDCALL icvGetRectSubPix_##flavor##_C1R                         \
( const srctype* src, int src_step, CvSize src_size,                        \
  dsttype* dst, int dst_step, CvSize win_size, CvPoint2D32f center )        \
{                                                                           \
    CvPoint ip;                                                             \
    worktype  a11, a12, a21, a22, b1, b2;                                   \
    float a, b;                                                             \
    int i, j;                                                               \
                                                                            \
    center.x -= (win_size.width-1)*0.5f;                                    \
    center.y -= (win_size.height-1)*0.5f;                                   \
                                                                            \
    ip.x = cvFloor( center.x );                                             \
    ip.y = cvFloor( center.y );                                             \
                                                                            \
    a = center.x - ip.x;                                                    \
    b = center.y - ip.y;                                                    \
    a11 = scale_macro((1.f-a)*(1.f-b));                                     \
    a12 = scale_macro(a*(1.f-b));                                           \
    a21 = scale_macro((1.f-a)*b);                                           \
    a22 = scale_macro(a*b);                                                 \
    b1 = scale_macro(1.f - b);                                              \
    b2 = scale_macro(b);                                                    \
                                                                            \
    src_step /= sizeof(src[0]);                                             \
    dst_step /= sizeof(dst[0]);                                             \
                                                                            \
    if( 0 <= ip.x && ip.x + win_size.width < src_size.width &&              \
        0 <= ip.y && ip.y + win_size.height < src_size.height )             \
    {                                                                       \
        /* extracted rectangle is totally inside the image */               \
        src += ip.y * src_step + ip.x;                                      \
                                                                            \
        for( i = 0; i < win_size.height; i++, src += src_step,              \
                                              dst += dst_step )             \
        {                                                                   \
            for( j = 0; j <= win_size.width - 2; j += 2 )                   \
            {                                                               \
                worktype s0 = cast_macro(src[j])*a11 +                      \
                              cast_macro(src[j+1])*a12 +                    \
                              cast_macro(src[j+src_step])*a21 +             \
                              cast_macro(src[j+src_step+1])*a22;            \
                worktype s1 = cast_macro(src[j+1])*a11 +                    \
                              cast_macro(src[j+2])*a12 +                    \
                              cast_macro(src[j+src_step+1])*a21 +           \
                              cast_macro(src[j+src_step+2])*a22;            \
                                                                            \
                dst[j] = (dsttype)cast_macro2(s0);                          \
                dst[j+1] = (dsttype)cast_macro2(s1);                        \
            }                                                               \
                                                                            \
            for( ; j < win_size.width; j++ )                                \
            {                                                               \
                worktype s0 = cast_macro(src[j])*a11 +                      \
                              cast_macro(src[j+1])*a12 +                    \
                              cast_macro(src[j+src_step])*a21 +             \
                              cast_macro(src[j+src_step+1])*a22;            \
                                                                            \
                dst[j] = (dsttype)cast_macro2(s0);                          \
            }                                                               \
        }                                                                   \
    }                                                                       \
    else                                                                    \
    {                                                                       \
        CvRect r;                                                           \
                                                                            \
        src = (const srctype*)icvAdjustRect( src, src_step*sizeof(*src),    \
                               sizeof(*src), src_size, win_size,ip, &r);    \
                                                                            \
        for( i = 0; i < win_size.height; i++, dst += dst_step )             \
        {                                                                   \
            const srctype *src2 = src + src_step;                           \
                                                                            \
            if( i < r.y || i >= r.height )                                  \
                src2 -= src_step;                                           \
                                                                            \
            for( j = 0; j < r.x; j++ )                                      \
            {                                                               \
                worktype s0 = cast_macro(src[r.x])*b1 +                     \
                              cast_macro(src2[r.x])*b2;                     \
                                                                            \
                dst[j] = (dsttype)cast_macro2(s0);                          \
            }                                                               \
                                                                            \
            for( ; j < r.width; j++ )                                       \
            {                                                               \
                worktype s0 = cast_macro(src[j])*a11 +                      \
                              cast_macro(src[j+1])*a12 +                    \
                              cast_macro(src2[j])*a21 +                     \
                              cast_macro(src2[j+1])*a22;                    \
                                                                            \
                dst[j] = (dsttype)cast_macro2(s0);                          \
            }                                                               \
                                                                            \
            for( ; j < win_size.width; j++ )                                \
            {                                                               \
                worktype s0 = cast_macro(src[r.width])*b1 +                 \
                              cast_macro(src2[r.width])*b2;                 \
                                                                            \
                dst[j] = (dsttype)cast_macro2(s0);                          \
            }                                                               \
                                                                            \
            if( i < r.height )                                              \
                src = src2;                                                 \
        }                                                                   \
    }                                                                       \
                                                                            \
    return CV_OK;                                                           \
}


///////////////////////////////
static const void*
icvAdjustRect( const void* srcptr, int src_step, int pix_size,
               CvSize src_size, CvSize win_size,
               CvPoint ip, CvRect* pRect )
{
    CvRect rect;
    const char* src = (const char*)srcptr;

    if( ip.x >= 0 )
    {
        src += ip.x*pix_size;
        rect.x = 0;
    }
    else
    {
        rect.x = -ip.x;
        if( rect.x > win_size.width )
            rect.x = win_size.width;
    }

    if( ip.x + win_size.width < src_size.width )
        rect.width = win_size.width;
    else
    {
        rect.width = src_size.width - ip.x - 1;
        if( rect.width < 0 )
        {
            src += rect.width*pix_size;
            rect.width = 0;
        }
        assert( rect.width <= win_size.width );
    }

    if( ip.y >= 0 )
    {
        src += ip.y * src_step;
        rect.y = 0;
    }
    else
        rect.y = -ip.y;

    if( ip.y + win_size.height < src_size.height )
        rect.height = win_size.height;
    else
    {
        rect.height = src_size.height - ip.y - 1;
        if( rect.height < 0 )
        {
            src += rect.height*src_step;
            rect.height = 0;
        }
    }

    *pRect = rect;
    return src - rect.x*pix_size;
}
const float icv8x32fTab_cv[] =
{
    -256.f, -255.f, -254.f, -253.f, -252.f, -251.f, -250.f, -249.f,
    -248.f, -247.f, -246.f, -245.f, -244.f, -243.f, -242.f, -241.f,
    -240.f, -239.f, -238.f, -237.f, -236.f, -235.f, -234.f, -233.f,
    -232.f, -231.f, -230.f, -229.f, -228.f, -227.f, -226.f, -225.f,
    -224.f, -223.f, -222.f, -221.f, -220.f, -219.f, -218.f, -217.f,
    -216.f, -215.f, -214.f, -213.f, -212.f, -211.f, -210.f, -209.f,
    -208.f, -207.f, -206.f, -205.f, -204.f, -203.f, -202.f, -201.f,
    -200.f, -199.f, -198.f, -197.f, -196.f, -195.f, -194.f, -193.f,
    -192.f, -191.f, -190.f, -189.f, -188.f, -187.f, -186.f, -185.f,
    -184.f, -183.f, -182.f, -181.f, -180.f, -179.f, -178.f, -177.f,
    -176.f, -175.f, -174.f, -173.f, -172.f, -171.f, -170.f, -169.f,
    -168.f, -167.f, -166.f, -165.f, -164.f, -163.f, -162.f, -161.f,
    -160.f, -159.f, -158.f, -157.f, -156.f, -155.f, -154.f, -153.f,
    -152.f, -151.f, -150.f, -149.f, -148.f, -147.f, -146.f, -145.f,
    -144.f, -143.f, -142.f, -141.f, -140.f, -139.f, -138.f, -137.f,
    -136.f, -135.f, -134.f, -133.f, -132.f, -131.f, -130.f, -129.f,
    -128.f, -127.f, -126.f, -125.f, -124.f, -123.f, -122.f, -121.f,
    -120.f, -119.f, -118.f, -117.f, -116.f, -115.f, -114.f, -113.f,
    -112.f, -111.f, -110.f, -109.f, -108.f, -107.f, -106.f, -105.f,
    -104.f, -103.f, -102.f, -101.f, -100.f,  -99.f,  -98.f,  -97.f,
     -96.f,  -95.f,  -94.f,  -93.f,  -92.f,  -91.f,  -90.f,  -89.f,
     -88.f,  -87.f,  -86.f,  -85.f,  -84.f,  -83.f,  -82.f,  -81.f,
     -80.f,  -79.f,  -78.f,  -77.f,  -76.f,  -75.f,  -74.f,  -73.f,
     -72.f,  -71.f,  -70.f,  -69.f,  -68.f,  -67.f,  -66.f,  -65.f,
     -64.f,  -63.f,  -62.f,  -61.f,  -60.f,  -59.f,  -58.f,  -57.f,
     -56.f,  -55.f,  -54.f,  -53.f,  -52.f,  -51.f,  -50.f,  -49.f,
     -48.f,  -47.f,  -46.f,  -45.f,  -44.f,  -43.f,  -42.f,  -41.f,
     -40.f,  -39.f,  -38.f,  -37.f,  -36.f,  -35.f,  -34.f,  -33.f,
     -32.f,  -31.f,  -30.f,  -29.f,  -28.f,  -27.f,  -26.f,  -25.f,
     -24.f,  -23.f,  -22.f,  -21.f,  -20.f,  -19.f,  -18.f,  -17.f,
     -16.f,  -15.f,  -14.f,  -13.f,  -12.f,  -11.f,  -10.f,   -9.f,
      -8.f,   -7.f,   -6.f,   -5.f,   -4.f,   -3.f,   -2.f,   -1.f,
       0.f,    1.f,    2.f,    3.f,    4.f,    5.f,    6.f,    7.f,
       8.f,    9.f,   10.f,   11.f,   12.f,   13.f,   14.f,   15.f,
      16.f,   17.f,   18.f,   19.f,   20.f,   21.f,   22.f,   23.f,
      24.f,   25.f,   26.f,   27.f,   28.f,   29.f,   30.f,   31.f,
      32.f,   33.f,   34.f,   35.f,   36.f,   37.f,   38.f,   39.f,
      40.f,   41.f,   42.f,   43.f,   44.f,   45.f,   46.f,   47.f,
      48.f,   49.f,   50.f,   51.f,   52.f,   53.f,   54.f,   55.f,
      56.f,   57.f,   58.f,   59.f,   60.f,   61.f,   62.f,   63.f,
      64.f,   65.f,   66.f,   67.f,   68.f,   69.f,   70.f,   71.f,
      72.f,   73.f,   74.f,   75.f,   76.f,   77.f,   78.f,   79.f,
      80.f,   81.f,   82.f,   83.f,   84.f,   85.f,   86.f,   87.f,
      88.f,   89.f,   90.f,   91.f,   92.f,   93.f,   94.f,   95.f,
      96.f,   97.f,   98.f,   99.f,  100.f,  101.f,  102.f,  103.f,
     104.f,  105.f,  106.f,  107.f,  108.f,  109.f,  110.f,  111.f,
     112.f,  113.f,  114.f,  115.f,  116.f,  117.f,  118.f,  119.f,
     120.f,  121.f,  122.f,  123.f,  124.f,  125.f,  126.f,  127.f,
     128.f,  129.f,  130.f,  131.f,  132.f,  133.f,  134.f,  135.f,
     136.f,  137.f,  138.f,  139.f,  140.f,  141.f,  142.f,  143.f,
     144.f,  145.f,  146.f,  147.f,  148.f,  149.f,  150.f,  151.f,
     152.f,  153.f,  154.f,  155.f,  156.f,  157.f,  158.f,  159.f,
     160.f,  161.f,  162.f,  163.f,  164.f,  165.f,  166.f,  167.f,
     168.f,  169.f,  170.f,  171.f,  172.f,  173.f,  174.f,  175.f,
     176.f,  177.f,  178.f,  179.f,  180.f,  181.f,  182.f,  183.f,
     184.f,  185.f,  186.f,  187.f,  188.f,  189.f,  190.f,  191.f,
     192.f,  193.f,  194.f,  195.f,  196.f,  197.f,  198.f,  199.f,
     200.f,  201.f,  202.f,  203.f,  204.f,  205.f,  206.f,  207.f,
     208.f,  209.f,  210.f,  211.f,  212.f,  213.f,  214.f,  215.f,
     216.f,  217.f,  218.f,  219.f,  220.f,  221.f,  222.f,  223.f,
     224.f,  225.f,  226.f,  227.f,  228.f,  229.f,  230.f,  231.f,
     232.f,  233.f,  234.f,  235.f,  236.f,  237.f,  238.f,  239.f,
     240.f,  241.f,  242.f,  243.f,  244.f,  245.f,  246.f,  247.f,
     248.f,  249.f,  250.f,  251.f,  252.f,  253.f,  254.f,  255.f,
     256.f,  257.f,  258.f,  259.f,  260.f,  261.f,  262.f,  263.f,
     264.f,  265.f,  266.f,  267.f,  268.f,  269.f,  270.f,  271.f,
     272.f,  273.f,  274.f,  275.f,  276.f,  277.f,  278.f,  279.f,
     280.f,  281.f,  282.f,  283.f,  284.f,  285.f,  286.f,  287.f,
     288.f,  289.f,  290.f,  291.f,  292.f,  293.f,  294.f,  295.f,
     296.f,  297.f,  298.f,  299.f,  300.f,  301.f,  302.f,  303.f,
     304.f,  305.f,  306.f,  307.f,  308.f,  309.f,  310.f,  311.f,
     312.f,  313.f,  314.f,  315.f,  316.f,  317.f,  318.f,  319.f,
     320.f,  321.f,  322.f,  323.f,  324.f,  325.f,  326.f,  327.f,
     328.f,  329.f,  330.f,  331.f,  332.f,  333.f,  334.f,  335.f,
     336.f,  337.f,  338.f,  339.f,  340.f,  341.f,  342.f,  343.f,
     344.f,  345.f,  346.f,  347.f,  348.f,  349.f,  350.f,  351.f,
     352.f,  353.f,  354.f,  355.f,  356.f,  357.f,  358.f,  359.f,
     360.f,  361.f,  362.f,  363.f,  364.f,  365.f,  366.f,  367.f,
     368.f,  369.f,  370.f,  371.f,  372.f,  373.f,  374.f,  375.f,
     376.f,  377.f,  378.f,  379.f,  380.f,  381.f,  382.f,  383.f,
     384.f,  385.f,  386.f,  387.f,  388.f,  389.f,  390.f,  391.f,
     392.f,  393.f,  394.f,  395.f,  396.f,  397.f,  398.f,  399.f,
     400.f,  401.f,  402.f,  403.f,  404.f,  405.f,  406.f,  407.f,
     408.f,  409.f,  410.f,  411.f,  412.f,  413.f,  414.f,  415.f,
     416.f,  417.f,  418.f,  419.f,  420.f,  421.f,  422.f,  423.f,
     424.f,  425.f,  426.f,  427.f,  428.f,  429.f,  430.f,  431.f,
     432.f,  433.f,  434.f,  435.f,  436.f,  437.f,  438.f,  439.f,
     440.f,  441.f,  442.f,  443.f,  444.f,  445.f,  446.f,  447.f,
     448.f,  449.f,  450.f,  451.f,  452.f,  453.f,  454.f,  455.f,
     456.f,  457.f,  458.f,  459.f,  460.f,  461.f,  462.f,  463.f,
     464.f,  465.f,  466.f,  467.f,  468.f,  469.f,  470.f,  471.f,
     472.f,  473.f,  474.f,  475.f,  476.f,  477.f,  478.f,  479.f,
     480.f,  481.f,  482.f,  483.f,  484.f,  485.f,  486.f,  487.f,
     488.f,  489.f,  490.f,  491.f,  492.f,  493.f,  494.f,  495.f,
     496.f,  497.f,  498.f,  499.f,  500.f,  501.f,  502.f,  503.f,
     504.f,  505.f,  506.f,  507.f,  508.f,  509.f,  510.f,  511.f,
};


#define CV_8TO32F(x)  icv8x32fTab_cv[(x)+256]
//static ICV_DEF_GET_RECT_SUB_PIX_FUNC( 32f, float, float, float, CV_NOP, CV_NOP, CV_NOP )
static ICV_DEF_GET_RECT_SUB_PIX_FUNC( 8u32f, uchar, float, float, CV_8TO32F, CV_NOP, CV_NOP )



static void
intersect( CvPoint2D32f pt, CvSize win_size, CvSize imgSize,
           CvPoint* min_pt, CvPoint* max_pt )
{
    CvPoint ipt;

    ipt.x = cvFloor( pt.x );
    ipt.y = cvFloor( pt.y );

    ipt.x -= win_size.width;
    ipt.y -= win_size.height;

    win_size.width = win_size.width * 2 + 1;
    win_size.height = win_size.height * 2 + 1;

    min_pt->x = MAX( 0, -ipt.x );
    min_pt->y = MAX( 0, -ipt.y );
    max_pt->x = MIN( win_size.width, imgSize.width - ipt.x );
    max_pt->y = MIN( win_size.height, imgSize.height - ipt.y );
}


static int icvMinimalPyramidSize( CvSize imgSize )
{
    return cvAlign(imgSize.width,8) * imgSize.height / 3;
}


static void
icvInitPyramidalAlgorithm( const CvMat* imgA, const CvMat* imgB,
                           CvMat* pyrA, CvMat* pyrB,
                           int level, CvTermCriteria * criteria,
                           int max_iters, int flags,
                           uchar *** imgI, uchar *** imgJ,
                           int **step, CvSize** size,
                           double **scale, cv::AutoBuffer<uchar>* buffer )
{
    const int ALIGN = 8;
    int pyrBytes, bufferBytes = 0, elem_size;
    int level1 = level + 1;

    int i;
    CvSize imgSize, levelSize;

    *imgI = *imgJ = 0;
    *step = 0;
    *scale = 0;
    *size = 0;

    /* check input arguments */
    if( ((flags & CV_LKFLOW_PYR_A_READY) != 0 && !pyrA) ||
        ((flags & CV_LKFLOW_PYR_B_READY) != 0 && !pyrB) )
        CV_Error( CV_StsNullPtr, "Some of the precomputed pyramids are missing" );

    if( level < 0 )
        CV_Error( CV_StsOutOfRange, "The number of pyramid levels is negative" );

    switch( criteria->type )
    {
    case CV_TERMCRIT_ITER:
        criteria->epsilon = 0.f;
        break;
    case CV_TERMCRIT_EPS:
        criteria->max_iter = max_iters;
        break;
    case CV_TERMCRIT_ITER | CV_TERMCRIT_EPS:
        break;
    default:
        assert( 0 );
        CV_Error( CV_StsBadArg, "Invalid termination criteria" );
    }

    /* compare squared values */
    criteria->epsilon *= criteria->epsilon;

    /* set pointers and step for every level */
    pyrBytes = 0;

    imgSize = cvGetSize(imgA);
    elem_size = CV_ELEM_SIZE(imgA->type);
    levelSize = imgSize;

    for( i = 1; i < level1; i++ )
    {
        levelSize.width = (levelSize.width + 1) >> 1;
        levelSize.height = (levelSize.height + 1) >> 1;

        int tstep = cvAlign(levelSize.width,ALIGN) * elem_size;
        pyrBytes += tstep * levelSize.height;
    }

    assert( pyrBytes <= imgSize.width * imgSize.height * elem_size * 4 / 3 );

    /* buffer_size = <size for patches> + <size for pyramids> */
    bufferBytes = (int)((level1 >= 0) * ((pyrA->data.ptr == 0) +
        (pyrB->data.ptr == 0)) * pyrBytes +
        (sizeof(imgI[0][0]) * 2 + sizeof(step[0][0]) +
         sizeof(size[0][0]) + sizeof(scale[0][0])) * level1);

    buffer->allocate( bufferBytes );

    *imgI = (uchar **) (uchar*)(*buffer);
    *imgJ = *imgI + level1;
    *step = (int *) (*imgJ + level1);
    *scale = (double *) (*step + level1);
    *size = (CvSize *)(*scale + level1);

    imgI[0][0] = imgA->data.ptr;
    imgJ[0][0] = imgB->data.ptr;
    step[0][0] = imgA->step;
    scale[0][0] = 1;
    size[0][0] = imgSize;

    if( level > 0 )
    {
        uchar *bufPtr = (uchar *) (*size + level1);
        uchar *ptrA = pyrA->data.ptr;
        uchar *ptrB = pyrB->data.ptr;

        if( !ptrA )
        {
            ptrA = bufPtr;
            bufPtr += pyrBytes;
        }

        if( !ptrB )
            ptrB = bufPtr;

        levelSize = imgSize;

        /* build pyramids for both frames */
        for( i = 1; i <= level; i++ )
        {
            int levelBytes;
            CvMat prev_level, next_level;

            levelSize.width = (levelSize.width + 1) >> 1;
            levelSize.height = (levelSize.height + 1) >> 1;

            size[0][i] = levelSize;
            step[0][i] = cvAlign( levelSize.width, ALIGN ) * elem_size;
            scale[0][i] = scale[0][i - 1] * 0.5;

            levelBytes = step[0][i] * levelSize.height;
            imgI[0][i] = (uchar *) ptrA;
            ptrA += levelBytes;

            if( !(flags & CV_LKFLOW_PYR_A_READY) )
            {
                prev_level = cvMat( size[0][i-1].height, size[0][i-1].width, CV_8UC1 );
                next_level = cvMat( size[0][i].height, size[0][i].width, CV_8UC1 );
                cvSetData( &prev_level, imgI[0][i-1], step[0][i-1] );
                cvSetData( &next_level, imgI[0][i], step[0][i] );
                cvPyrDown( &prev_level, &next_level );
            }

            imgJ[0][i] = (uchar *) ptrB;
            ptrB += levelBytes;

            if( !(flags & CV_LKFLOW_PYR_B_READY) )
            {
                prev_level = cvMat( size[0][i-1].height, size[0][i-1].width, CV_8UC1 );
                next_level = cvMat( size[0][i].height, size[0][i].width, CV_8UC1 );
                cvSetData( &prev_level, imgJ[0][i-1], step[0][i-1] );
                cvSetData( &next_level, imgJ[0][i], step[0][i] );
                cvPyrDown( &prev_level, &next_level );
            }
        }
    }
}


/* compute dI/dx and dI/dy */
static void
icvCalcIxIy_32f( const float* src, int src_step, float* dstX, float* dstY, int dst_step,
                 CvSize src_size, const float* smooth_k, float* buffer0 )
{
    int src_width = src_size.width, dst_width = src_size.width-2;
    int x, height = src_size.height - 2;
    float* buffer1 = buffer0 + src_width;

    src_step /= sizeof(src[0]);
    dst_step /= sizeof(dstX[0]);

    for( ; height--; src += src_step, dstX += dst_step, dstY += dst_step )
    {
        const float* src2 = src + src_step;
        const float* src3 = src + src_step*2;

        for( x = 0; x < src_width; x++ )
        {
            float t0 = (src3[x] + src[x])*smooth_k[0] + src2[x]*smooth_k[1];
            float t1 = src3[x] - src[x];
            buffer0[x] = t0; buffer1[x] = t1;
        }

        for( x = 0; x < dst_width; x++ )
        {
            float t0 = buffer0[x+2] - buffer0[x];
            float t1 = (buffer1[x] + buffer1[x+2])*smooth_k[0] + buffer1[x+1]*smooth_k[1];
            dstX[x] = t0; dstY[x] = t1;
        }
    }
}


static float
idiapcvAverage_32f(const float* src, int src_step, CvSize src_size)
{
    //int src_width = src_size.width;
    float sum = 0;
    src_step /= sizeof (src[0]);
    for (int y = 0; y < src_size.height; y++) {
        const float* src2 = src + y*src_step;
        for (int x = 0; x < src_size.width; x++) {
            sum += src2[x];
        }
    }
    return sum / src_size.width / src_size.height;
}


namespace idiapcv
{


        class BlockedRange
        {
        public:
            BlockedRange() : _begin(0), _end(0), _grainsize(0) {}
            BlockedRange(int b, int e, int g=1) : _begin(b), _end(e), _grainsize(g) {}
            int begin() const { return _begin; }
            int end() const { return _end; }
            int grainsize() const { return _grainsize; }

        protected:
            int _begin, _end, _grainsize;
        };

        template<typename Body> static inline
        void parallel_for( const BlockedRange& range, const Body& body )
        {
            body(range);
        }

        template<typename Iterator, typename Body> static inline
        void parallel_do( Iterator first, Iterator last, const Body& body )
        {
            for( ; first != last; ++first )
                body(*first);
        }

        class Split {};

        template<typename Body> static inline
        void parallel_reduce( const BlockedRange& range, Body& body )
        {
            body(range);
        }

    //        typedef std::vector<Rect> ConcurrentRectVector;








struct LKTrackerInvoker
{
    LKTrackerInvoker( const CvMat* _imgI, const CvMat* _imgJ,
                      const CvPoint2D32f* _featuresA,
                      CvPoint2D32f* _featuresB,
                      char* _status, float* _error,
                      CvTermCriteria _criteria,
                      CvSize _winSize, int _level, int _flags,
                      float _gMin,
                      int _staticWindowSize, float _staticThreshold,
                      int _sizeFactor)
    {
        imgI = _imgI;
        imgJ = _imgJ;
        featuresA = _featuresA;
        featuresB = _featuresB;
        status = _status;
        error = _error;
        criteria = _criteria;
        winSize = _winSize;
        level = _level;
        flags = _flags;
        gMin = _gMin;
        staticWindowSize = _staticWindowSize;
        staticThreshold = _staticThreshold;
        sizeFactor = _sizeFactor;
    }

    void operator()(const BlockedRange& range) const
    {
        static const float smoothKernel[] = { 0.09375, 0.3125, 0.09375 };  // 3/32, 10/32, 3/32

        int i, i1 = range.begin(), i2 = range.end();

        CvSize patchSize = cvSize( winSize.width * 2 + 1, winSize.height * 2 + 1 );
        int patchLen = patchSize.width * patchSize.height;
        int srcPatchLen = (patchSize.width + 2)*(patchSize.height + 2);

        cv::AutoBuffer<float> buf(patchLen*3 + srcPatchLen);
        float* patchI = buf;
        float* patchJ = patchI + srcPatchLen;
        float* Ix = patchJ + patchLen;
        float* Iy = Ix + patchLen;
        float scaleL = 1.f/(1 << level);
        CvSize levelSize = cvGetMatSize(imgI);

        // find flow for each given point
        for( i = i1; i < i2; i++ )
        {
            CvPoint2D32f v;
            CvPoint minI, maxI, minJ, maxJ;
            CvSize isz, jsz;
            int pt_status;
            CvPoint2D32f u;
            CvPoint prev_minJ = { -1, -1 }, prev_maxJ = { -1, -1 };
            double Gxx = 0, Gxy = 0, Gyy = 0, D = 0, minEig = 0;
            float prev_mx = 0, prev_my = 0;
            int j, x, y;

            v.x = featuresB[i].x*2;
            v.y = featuresB[i].y*2;

            pt_status = status[i];
            if( !pt_status )
                continue;

            minI = maxI = minJ = maxJ = cvPoint(0, 0);

            u.x = featuresA[i].x * scaleL;
            u.y = featuresA[i].y * scaleL;

            intersect( u, winSize, levelSize, &minI, &maxI );
            isz = jsz = cvSize(maxI.x - minI.x + 2, maxI.y - minI.y + 2);
            u.x += (minI.x - (patchSize.width - maxI.x + 1))*0.5f;
            u.y += (minI.y - (patchSize.height - maxI.y + 1))*0.5f;

            if( isz.width < 3 || isz.height < 3 ||
                icvGetRectSubPix_8u32f_C1R( imgI->data.ptr, imgI->step, levelSize,
                                            patchI, isz.width*sizeof(patchI[0]), isz, u ) < 0 )
            {
                // point is outside the first image. take the next
                status[i] = 0;
                continue;
            }

            if (level == 0 && staticWindowSize > 0 && staticThreshold < 255 && staticThreshold > 0) {
              // TODO should do as above (with the intersect calls etc), currently we handle only the same window size

                icvGetRectSubPix_8u32f_C1R(imgJ->data.ptr, imgJ->step, levelSize, patchJ, jsz.width * sizeof (patchI[0]), isz, u);
                float averageAtPreviousTime = idiapcvAverage_32f(patchI, isz.width*sizeof(patchI[0]), isz);
                float averageAtNextTime = idiapcvAverage_32f(patchJ, jsz.width*sizeof(patchJ[0]), jsz);
                if (fabs(averageAtPreviousTime - averageAtNextTime) <= staticThreshold) {
                    featuresB[i] = featuresA[i];
                    featuresB[i].x += 0.00010203; // to distinguish from pure 0
                    error[i] = 0;
                    continue;
                }

                /*if (staticWindowSize > winSize.width || staticWindowSize > winSize.height) {
                    throw "unsupported static window size bigger than window size";
                }
                CvSize stwSize = cvSize( staticWindowSize * 2 + 1, staticWindowSize * 2 + 1 );
                icvGetRectSubPix_8u32f_C1R(imgI->data.ptr, imgI->step, levelSize, patchI, isz.width * sizeof (patchI[0]), stwSize, u);
                icvGetRectSubPix_8u32f_C1R(imgJ->data.ptr, imgJ->step, levelSize, patchJ, jsz.width * sizeof (patchI[0]), stwSize, u);
                float averageAtPreviousTime = idiapcvAverage_32f(patchI, isz.width*sizeof(patchI[0]), stwSize);
                float averageAtNextTime = idiapcvAverage_32f(patchJ, jsz.width*sizeof(patchJ[0]), stwSize);
                icvGetRectSubPix_8u32f_C1R(imgI->data.ptr, imgI->step, levelSize, patchI, isz.width * sizeof (patchI[0]), isz, u);
                 */
            }

            icvCalcIxIy_32f( patchI, isz.width*sizeof(patchI[0]), Ix, Iy,
                             (isz.width-2)*sizeof(patchI[0]), isz, smoothKernel, patchJ );

            for( j = 0; j < criteria.max_iter; j++ )
            {
                double bx = 0, by = 0;
                float mx, my;
                CvPoint2D32f _v;

                intersect( v, winSize, levelSize, &minJ, &maxJ );

                minJ.x = MAX( minJ.x, minI.x );
                minJ.y = MAX( minJ.y, minI.y );

                maxJ.x = MIN( maxJ.x, maxI.x );
                maxJ.y = MIN( maxJ.y, maxI.y );

                jsz = cvSize(maxJ.x - minJ.x, maxJ.y - minJ.y);

                _v.x = v.x + (minJ.x - (patchSize.width - maxJ.x + 1))*0.5f;
                _v.y = v.y + (minJ.y - (patchSize.height - maxJ.y + 1))*0.5f;

                if( jsz.width < 1 || jsz.height < 1 ||
                    icvGetRectSubPix_8u32f_C1R( imgJ->data.ptr, imgJ->step, levelSize, patchJ,
                                                jsz.width*sizeof(patchJ[0]), jsz, _v ) < 0 )
                {
                    // point is outside of the second image. take the next
                    pt_status = 0;
                    break;
                }

                if( maxJ.x == prev_maxJ.x && maxJ.y == prev_maxJ.y &&
                    minJ.x == prev_minJ.x && minJ.y == prev_minJ.y )
                {
                    for( y = 0; y < jsz.height; y++ )
                    {
                        const float* pi = patchI +
                        (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
                        const float* pj = patchJ + y*jsz.width;
                        const float* ix = Ix +
                        (y + minJ.y - minI.y)*(isz.width-2) + minJ.x - minI.x;
                        const float* iy = Iy + (ix - Ix);

                        for( x = 0; x < jsz.width; x++ )
                        {
                            double t0 = pi[x] - pj[x];
                            bx += t0 * ix[x];
                            by += t0 * iy[x];
                        }
                    }
                }
                else
                {
                    Gxx = Gyy = 0;
                    Gxy = 0;
                    for( y = 0; y < jsz.height; y++ )
                    {
                        const float* pi = patchI +
                        (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
                        const float* pj = patchJ + y*jsz.width;
                        const float* ix = Ix +
                        (y + minJ.y - minI.y)*(isz.width-2) + minJ.x - minI.x;
                        const float* iy = Iy + (ix - Ix);

                        for( x = 0; x < jsz.width; x++ )
                        {
                            double t = pi[x] - pj[x];
                            bx += (double) (t * ix[x]);
                            by += (double) (t * iy[x]);
                            Gxx += ix[x] * ix[x];
                            Gxy += ix[x] * iy[x];
                            Gyy += iy[x] * iy[x];
                        }
                    }
                    {
                        float min = gMin * gMin * jsz.width * jsz.height;
                        if (Gxx < min && Gyy < min) {
                            float delta = min - fmax(Gxx, Gyy);
                            Gxx += delta;
                            Gyy += delta;
                        }
                    }

                    D = Gxx * Gyy - Gxy * Gxy;
                    if( D < DBL_EPSILON )
                    {
                        pt_status = 0;
                        break;
                    }

                    // Adi Shavit - 2008.05
                    if( flags & CV_LKFLOW_GET_MIN_EIGENVALS )
                        minEig = (Gyy + Gxx - sqrt((Gxx-Gyy)*(Gxx-Gyy) + 4.*Gxy*Gxy))/(2*jsz.height*jsz.width);

                    D = 1. / D;

                    prev_minJ = minJ;
                    prev_maxJ = maxJ;
                }

                mx = (float) ((Gyy * bx - Gxy * by) * D);
                my = (float) ((Gxx * by - Gxy * bx) * D);

                v.x += mx;
                v.y += my;

                if( mx * mx + my * my < criteria.epsilon )
                    break;

                if( j > 0 && fabs(mx + prev_mx) < 0.01 && fabs(my + prev_my) < 0.01 )
                {
                    v.x -= mx*0.5f;
                    v.y -= my*0.5f;
                    break;
                }
                prev_mx = mx;
                prev_my = my;
            }

            featuresB[i] = v;
            status[i] = (char)pt_status;
            if( level == 0 && error && pt_status )
            {
                // calc error
                double err = 0;
                if( flags & CV_LKFLOW_GET_MIN_EIGENVALS )
                    err = minEig;
                else
                {
                    for( y = 0; y < jsz.height; y++ )
                    {
                        const float* pi = patchI +
                        (y + minJ.y - minI.y + 1)*isz.width + minJ.x - minI.x + 1;
                        const float* pj = patchJ + y*jsz.width;

                        for( x = 0; x < jsz.width; x++ )
                        {
                            double t = pi[x] - pj[x];
                            err += t * t;
                        }
                    }
                    err = sqrt(err);
                }
                error[i] = (float)err;
            }
        } // end of point processing loop (i)
    }

    const CvMat* imgI;
    const CvMat* imgJ;
    const CvPoint2D32f* featuresA;
    CvPoint2D32f* featuresB;
    char* status;
    float* error;
    CvTermCriteria criteria;
    CvSize winSize;
    int level;
    int flags;
    float gMin;
    int staticWindowSize;
    float staticThreshold;
    int sizeFactor;
};


}


CV_IMPL void
cvpCalcOpticalFlowPyrLK(const void* arrA, const void* arrB,
                        void* pyrarrA, void* pyrarrB,
                        const CvPoint2D32f * featuresA,
                        CvPoint2D32f * featuresB,
                        int count, CvSize winSize, int level,
                        char *status, float *error,
                        CvTermCriteria criteria, int flags,
                        float Gmin,
                        int static_window_size,
                        float static_threshold,
                        int size_factor)
{
    cv::AutoBuffer<uchar> pyrBuffer;
    cv::AutoBuffer<uchar> buffer;
    cv::AutoBuffer<char> _status;

    const int MAX_ITERS = 100;

    CvMat stubA, *imgA = (CvMat*)arrA;
    CvMat stubB, *imgB = (CvMat*)arrB;
    CvMat pstubA, *pyrA = (CvMat*)pyrarrA;
    CvMat pstubB, *pyrB = (CvMat*)pyrarrB;
    CvSize imgSize;

    uchar **imgI = 0;
    uchar **imgJ = 0;
    int *step = 0;
    double *scale = 0;
    CvSize* size = 0;

    int i, l;

    imgA = cvGetMat( imgA, &stubA );
    imgB = cvGetMat( imgB, &stubB );

    if( CV_MAT_TYPE( imgA->type ) != CV_8UC1 )
        CV_Error( CV_StsUnsupportedFormat, "" );

    if( !CV_ARE_TYPES_EQ( imgA, imgB ))
        CV_Error( CV_StsUnmatchedFormats, "" );

    if( !CV_ARE_SIZES_EQ( imgA, imgB ))
        CV_Error( CV_StsUnmatchedSizes, "" );

    if( imgA->step != imgB->step )
        CV_Error( CV_StsUnmatchedSizes, "imgA and imgB must have equal steps" );

    imgSize = cvGetMatSize( imgA );

    if( pyrA )
    {
        pyrA = cvGetMat( pyrA, &pstubA );

        if( pyrA->step*pyrA->height < icvMinimalPyramidSize( imgSize ) )
            CV_Error( CV_StsBadArg, "pyramid A has insufficient size" );
    }
    else
    {
        pyrA = &pstubA;
        pyrA->data.ptr = 0;
    }

    if( pyrB )
    {
        pyrB = cvGetMat( pyrB, &pstubB );

        if( pyrB->step*pyrB->height < icvMinimalPyramidSize( imgSize ) )
            CV_Error( CV_StsBadArg, "pyramid B has insufficient size" );
    }
    else
    {
        pyrB = &pstubB;
        pyrB->data.ptr = 0;
    }

    if( count == 0 )
        return;

    if( !featuresA || !featuresB )
        CV_Error( CV_StsNullPtr, "Some of arrays of point coordinates are missing" );

    if( count < 0 )
        CV_Error( CV_StsOutOfRange, "The number of tracked points is negative or zero" );

    if( winSize.width <= 1 || winSize.height <= 1 )
        CV_Error( CV_StsBadSize, "Invalid search window size" );

    icvInitPyramidalAlgorithm( imgA, imgB, pyrA, pyrB,
        level, &criteria, MAX_ITERS, flags,
        &imgI, &imgJ, &step, &size, &scale, &pyrBuffer );

    if( !status )
    {
        _status.allocate(count);
        status = _status;
    }

    memset( status, 1, count );
    if( error )
        memset( error, 0, count*sizeof(error[0]) );

    if( !(flags & CV_LKFLOW_INITIAL_GUESSES) )
        memcpy( featuresB, featuresA, count*sizeof(featuresA[0]));

    for( i = 0; i < count; i++ )
    {
        featuresB[i].x = (float)(featuresB[i].x * scale[level] * 0.5);
        featuresB[i].y = (float)(featuresB[i].y * scale[level] * 0.5);
    }

    /* do processing from top pyramid level (smallest image)
       to the bottom (original image) */
    for( l = level; l >= 0; l-- )
    {
        CvMat imgI_l, imgJ_l;
        cvInitMatHeader(&imgI_l, size[l].height, size[l].width, imgA->type, imgI[l], step[l]);
        cvInitMatHeader(&imgJ_l, size[l].height, size[l].width, imgB->type, imgJ[l], step[l]);

        CvSize currentSize;
        currentSize.width = winSize.width;
        currentSize.height = winSize.height;
        if (size_factor) {
            currentSize.width /= (size_factor << (l / 2));
            currentSize.height /= (size_factor << (l / 2));
        }
        idiapcv::parallel_for(idiapcv::BlockedRange(0, count),
                         idiapcv::LKTrackerInvoker(&imgI_l, &imgJ_l, featuresA,
                                                   featuresB, status, error,
                                                   criteria,
                                                   currentSize,
                                                   l, flags,
                                                   Gmin,
                                                   static_window_size, static_threshold,
                                                   size_factor));
    } // end of pyramid levels loop (l)
}



/* End of file. */
