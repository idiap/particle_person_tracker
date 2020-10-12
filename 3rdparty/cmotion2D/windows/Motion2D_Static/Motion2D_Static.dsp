# Microsoft Developer Studio Project File - Name="Motion2D_Static" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=Motion2D_Static - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "Motion2D_Static.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "Motion2D_Static.mak" CFG="Motion2D_Static - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "Motion2D_Static - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "Motion2D_Static - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "Motion2D_Static - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /I "..\..\src\inc" /I "..\..\src\cprim" /I "..\..\src\memoire" /I "..\..\include" /I "..\..\src\mat_gauss" /I "..\..\src\mat_sym" /D "_CONSOLE" /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "__NO_IMAGEIO_PNG_" /D "__NO_IMAGEIO_MPEG_" /YX /FD /TP /c
# ADD BASE RSC /l 0x40c /d "NDEBUG"
# ADD RSC /l 0x40c /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386 /out:"..\..\bin\WIN32\Motion2D.Static.exe"

!ELSEIF  "$(CFG)" == "Motion2D_Static - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /I "..\..\src\inc" /I "..\..\src\cprim" /I "..\..\src\memoire" /I "..\..\include" /I "..\..\src\mat_gauss" /I "..\..\src\mat_sym" /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /D "__NO_IMAGEIO_PNG_" /FR /YX /FD /GZ /TP /c
# ADD BASE RSC /l 0x40c /d "_DEBUG"
# ADD RSC /l 0x40c /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /out:"..\..\bin\WIN32\Motion2D.Static.exe" /pdbtype:sept

!ENDIF 

# Begin Target

# Name "Motion2D_Static - Win32 Release"
# Name "Motion2D_Static - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Group "src"

# PROP Default_Filter ""
# Begin Group "pyramide"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\pyramide\filt_gauss.c
# End Source File
# Begin Source File

SOURCE=..\..\src\pyramide\gradient.c
# End Source File
# Begin Source File

SOURCE=..\..\src\pyramide\multigr.c
# End Source File
# Begin Source File

SOURCE=..\..\src\pyramide\pyramide.c
# End Source File
# End Group
# Begin Group "memoire"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\memoire\memoire.c
# End Source File
# End Group
# Begin Group "mat_sym"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\mat_sym\arithm.c
# End Source File
# Begin Source File

SOURCE=..\..\src\mat_sym\inverse_mat_sym.c
# End Source File
# Begin Source File

SOURCE=..\..\src\mat_sym\invert.c
# End Source File
# Begin Source File

SOURCE=..\..\src\mat_sym\mtx_tool.c
# End Source File
# Begin Source File

SOURCE=..\..\src\mat_sym\resoud_mat_sym.c
# End Source File
# End Group
# Begin Group "interface"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\interface\CMotion2DEstimator.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\interface\CMotion2DModel.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\interface\CMotion2DPyramid.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\interface\CMotion2DWarping.cpp
# End Source File
# End Group
# Begin Group "image"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\image\CImageReader.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\image\CImageWriter.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\image\CMotion2DVideo_Mpeg2.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\image\CMpeg2Reader.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\image\CReader.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\image\CWriter.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\image\FieldVector.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\image\Motion2DImage_PNG.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\image\Motion2DImage_PNM.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\image\Motion2DImage_RAW.cpp
# End Source File
# End Group
# Begin Group "estimation"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\estimation\cog.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\covariance.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\estimate.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\estimate_aff.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\estimate_const.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\estimate_quad.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\im_spat_temp.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\irls.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\mem_est.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\para_mvt.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\RMRmod.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\variance.c
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\weights.c
# End Source File
# End Group
# Begin Group "cprim"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\cprim\acast.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\daarith.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\damem.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\faarith.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\fafirf.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\fafirf3.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\fafirf5.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\fafirf7.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\famem.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\saarith.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\ucaarith.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\ucamem.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\uiafirf3.c
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\uiamem.c
# End Source File
# End Group
# Begin Group "compense"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\compense\compense.c
# End Source File
# End Group
# End Group
# Begin Group "examples"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\examples\Motion2D.cpp
# End Source File
# End Group
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Group "include"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\include\CImageReader.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CImageWriter.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CMotion2D.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CMotion2DEstimator.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CMotion2DImage.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CMotion2DImage_base.cpp
# PROP Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=..\..\include\CMotion2DModel.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CMotion2DPyramid.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CMotion2DVideo_Mpeg2.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CMotion2DWarping.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CMpeg2Reader.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CReader.h
# End Source File
# Begin Source File

SOURCE=..\..\include\CWriter.h
# End Source File
# Begin Source File

SOURCE=..\..\include\FieldVector.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Motion2D.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Motion2DImage_PNG.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Motion2DImage_PNM.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Motion2DImage_RAW.h
# End Source File
# End Group
# Begin Group "src No. 1"

# PROP Default_Filter ""
# Begin Group "inc"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\inc\constant.h
# End Source File
# Begin Source File

SOURCE=..\..\src\inc\interplt.h
# End Source File
# Begin Source File

SOURCE=..\..\src\inc\macro.h
# End Source File
# Begin Source File

SOURCE=..\..\src\inc\type.h
# End Source File
# End Group
# Begin Group "pyramide No. 1"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\pyramide\filt_gauss.h
# End Source File
# Begin Source File

SOURCE=..\..\src\pyramide\gradient.h
# End Source File
# Begin Source File

SOURCE=..\..\src\pyramide\multigr.h
# End Source File
# Begin Source File

SOURCE=..\..\src\pyramide\pyramide.h
# End Source File
# End Group
# Begin Group "memoire No. 1"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\memoire\memoire.h
# End Source File
# End Group
# Begin Group "mat_sym No. 1"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\mat_sym\arithm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mat_sym\inverse_mat_sym.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mat_sym\invert.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mat_sym\mtx_tool.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mat_sym\resoud_mat_sym.h
# End Source File
# End Group
# Begin Group "estimation No. 1"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\estimation\cog.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\covariance.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\estimate.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\estimate_aff.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\estimate_const.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\estimate_quad.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\im_spat_temp.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\irls.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\mem_est.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\para_mvt.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\RMRmod.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\variance.h
# End Source File
# Begin Source File

SOURCE=..\..\src\estimation\weights.h
# End Source File
# End Group
# Begin Group "cprim No. 1"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\cprim\acast.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\daarith.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\damem.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\faarith.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\fafirf.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\fafirf3.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\fafirf5.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\fafirf7.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\famem.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\saarith.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\ucaarith.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\ucamem.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\uiafirf3.h
# End Source File
# Begin Source File

SOURCE=..\..\src\cprim\uiamem.h
# End Source File
# End Group
# Begin Group "compense No. 1"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\compense\compense.h
# End Source File
# End Group
# End Group
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# End Target
# End Project
