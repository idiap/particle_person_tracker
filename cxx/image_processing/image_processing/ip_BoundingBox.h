/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#ifndef IP_BOUNDING_BOX_HH
#define IP_BOUNDING_BOX_HH


#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "ip_Image.h"
#include "utils.h"

//#include "general.h"

/*
#ifndef min
/// The min function
#define	min(a,b) ((a) > (b) ? (b) : (a))
#endif

#ifndef max
/// The max function
#define	max(a,b) ((a) > (b) ? (a) : (b))
#endif
*/


namespace ImageProcessing  {

 /** 
     Basic class interface for representing bounding boxes.
      
    
      @author Jean-marc Odobez (Jean-Marc.Odobez@idiap.ch)
      @author Daniel Gatica-Perez (gatica@idiap.ch)
  */
  
  /*

    Bounding box   : 
         - simply defines the limit of a window in an image.
         - line numbers increasing from top to bottom
	 - column number increasing from left to right
	 NOTE :
	   - last line and last column are NOT included in bounding box
	   - we keep the same "axis representations" as with 
             pixels and points ;
	     That is, we always start by providing 
             X or COLUMN or WIDTH before Y or LINE or HEIGHT

	 * constructors :

	 - ip_BoundingBox   F()

	 - ip_BoundingBox   F(first_col,first_line,
	                      last_col,last_line)
            ATTENTION : LAST LINES AND LAST COLUMNS NOT INCLUDED

         - ip_BoundingBox   F(nb_columns,nb_lines)
           First lines and columns assumed to be 0

	 * Functions :

	 - init(first_col,first_line,
	                      last_col,last_line)
                ATTENTION : LAST LINES AND LAST COLUMNS NOT INCLUDED

	 - initPosAndSize(first_col,first_line,
	                  width,height);
               

         - initSize(width,height)

         - firstLine(),firstColumn(),lastLine(),lastColumn()
                ATTENTION : LAST LINES AND LAST COLUMNS NOT INCLUDED

	 - width(), height()

	 - translate(dep_co,dep_li)

	 - display(comment) 

	 - inside(column,line)

	 - bool empty()

	 - setEmpty()

	 - area()

	 - add(col,line) : adjust boundaries so that the point(col,line) lays inside the bounding box

	 - intersection(Bbox)

	 - outerBoundingBox(Bbox)

	 - draw(ip_Image<T> & Ima,T color,int Thickness=1){

 */


  class  ip_BoundingBox
    {
    public: // members : same as opencv, so that we can do casting
      int m_iFirstColumn; // named x in opencv 
      int m_iFirstLine; // named y in opencv
      int m_iWidth;
      int m_iHeight;
      

      //----
      ip_BoundingBox(){
	init(0,0,0,0);
      }

      //----
      ip_BoundingBox(int first_col,int first_line,
			     int last_col,int last_line){
	init(first_col,first_line,last_col,last_line);
      }

      //            ATTENTION : LAST LINES AND LAST COLUMNS NOT INCLUDED

      //----
      //     First lines and columns assumed to be 0
      ip_BoundingBox(int nb_columns,int nb_lines){
	init(0,0,nb_columns,nb_lines);
      }

      //----
      // Functions :
      //----
      // ATTENTION : LAST LINES AND LAST COLUMNS NOT INCLUDED
      inline virtual void init(int first_col,int first_line,int last_col,int last_line){
	m_iFirstLine=first_line; m_iFirstColumn=first_col;
	/*
	m_iHeight=last_line-first_line; //+1;
	m_iWidth=last_col-first_col; //+1;
	*/
	// important change: SD 2011-10-26
	m_iHeight=last_line-first_line+1;
	m_iWidth=last_col-first_col+1;
	m_iWidth=MAX(m_iWidth,0);
	m_iHeight=MAX(m_iHeight,0);
      }


      inline virtual void initPosAndSize(int first_col,int first_line,
					 int width,int height){
	m_iFirstLine=first_line; m_iFirstColumn=first_col;
	m_iWidth=width;  m_iHeight=height;
	/*
	m_iWidth=MAX(m_iWidth,0);
	m_iHeight=MAX(m_iHeight,0);
	*/
      }
               
      inline virtual void initCenterAndSize(int center_x,int center_y,
					 int width,int height){
	m_iFirstLine=center_y-height/2; m_iFirstColumn=center_x-width/2;
	m_iWidth=width;  m_iHeight=height;
	/*
	m_iWidth=MAX(m_iWidth,0);
	m_iHeight=MAX(m_iHeight,0);
	*/
      }
               
      inline virtual void initSize(int width,int height){
	m_iWidth=width;  m_iHeight=height;
	/*
	m_iWidth=MAX(m_iWidth,0);
	m_iHeight=MAX(m_iHeight,0);
	*/
      }

      //----
      inline virtual int  firstLine(){ return m_iFirstLine;}
      inline virtual int  firstColumn(){ return m_iFirstColumn; }
      //  ATTENTION : LAST LINES AND LAST COLUMNS NOT INCLUDED
      inline virtual int  lastLine(){ return m_iFirstLine+m_iHeight-1;}
      inline virtual int  lastColumn(){ return m_iFirstColumn+m_iWidth-1; }

      //-------
      inline virtual int  width(){ return m_iWidth;}
      inline virtual int  height(){ return m_iHeight; }

      //-------
      inline virtual void translate(int dep_co,int dep_li){
	m_iFirstLine+=dep_li;
	m_iFirstColumn+=dep_co;
      }

      //-------
      virtual void  display(const char *comment="",FILE *f=stdout){
	printf("\nBounding box : %s \n",comment);
	printf("  First line : %5d  First column : %5d  -- Height : %5d Width : %5d\n",
	       m_iFirstLine,m_iFirstColumn,m_iHeight,m_iWidth);
	printf("  Last  line : %5d  Last  column : %5d \n",
	       lastLine(),lastColumn());
	fflush(f);
      }
      
      //-------
      inline virtual bool inside(int column,int line){
	if(line>=m_iFirstLine && column>=m_iFirstColumn && line <= lastLine() && column <= lastColumn())
	  return true;
	else
	  return false;
      }

      //-------
      inline virtual int area(){
	return height()*width();
      }

      //-------
      inline virtual bool empty(){
	if(width()==0 || height()==0)
	  return true;
	else
	  return false;
      }
      
      //-------
      inline virtual void setEmpty(){
	init(0,0,0,0);
      }

       //-------
      inline virtual void add(int col,int li){
	if(empty()){
	  m_iFirstColumn=col; m_iWidth = 1; m_iFirstLine=li; m_iHeight=1;
	}
	else {
	  if(col<m_iFirstColumn) m_iFirstColumn=col;
	  if(col>lastColumn()) m_iWidth += (col-lastColumn());
	  if(li<m_iFirstLine) m_iFirstLine=li;
	  if(li>lastLine()) m_iHeight += (li-lastLine());
	}
      }
      
      //-------
      inline virtual void add(float col,float  li){
	add((int)round(col),(int)round(li));
      }
      
      inline virtual int centerX() { return m_iFirstColumn+m_iWidth/2; };
      inline virtual int centerY() { return m_iFirstLine+m_iHeight/2; };

      //-------
      inline virtual void intersection(ip_BoundingBox & BBox2){
	ip_BoundingBox B(
			 MAX(firstColumn(),BBox2.firstColumn()),
			 MAX(firstLine(),BBox2.firstLine()),
			 MIN(lastColumn(),BBox2.lastColumn()),
			 MIN(lastLine(),BBox2.lastLine())
			 );
	(*this)=B;
      }

      inline virtual bool isOverlapping(ip_BoundingBox& bb)
      {
	if ((firstColumn() < bb.firstColumn() && lastColumn() < bb.firstColumn()) || (bb.firstColumn() < firstColumn() && bb.lastColumn() < firstColumn()))
	  return false;
	if ((firstLine() < bb.firstLine() && lastLine() < bb.firstLine()) || (bb.firstLine() < firstLine() && bb.lastLine() < firstLine()))
	  return false;
        return true;
      }

      inline virtual float precision(ip_BoundingBox& ground_truth)
      {
	// "this" object is detection result
	ip_BoundingBox tmp = *this;
	tmp.intersection(ground_truth);
	int a = area();
	if (a>0)
	  return (float)tmp.area()/a;
	else
	  return 0;
      }

      inline virtual float recall(ip_BoundingBox& ground_truth)
      {
	// "this" object is detection result
	ip_BoundingBox tmp = *this;
	tmp.intersection(ground_truth);
	int a = ground_truth.area();
	if (a>0)
	  return (float)tmp.area()/a;
	else 
	  return 0;
      }

      inline virtual float f_measure(ip_BoundingBox& ground_truth)
      {
	if (!isOverlapping(ground_truth))
	  return 0;

	float res;
	/*
	float prec, rec, res;
	prec = precision(ground_truth);
	rec = recall(ground_truth);
	*/
	ip_BoundingBox intersec = *this;
	intersec.intersection(ground_truth);
	//printf("prec: %f  rec: %f\n", prec, rec);
	/*
	if (prec<=1e-10 && rec<=1e-10)
	  res=0;
	else
	*/
          //res = (2.0*prec*rec)/(prec+rec);
          res = (2.0*intersec.area())/(area()+ground_truth.area());
	return res;
      }


      //-------
      inline virtual void outerBoundingBox(ip_BoundingBox & BBox2){
	ip_BoundingBox B(MIN(firstColumn(),BBox2.firstColumn()),
			 MIN(firstLine(),BBox2.firstLine()),
			 MAX(lastColumn(),BBox2.lastColumn()),
			 MAX(lastLine(),BBox2.lastLine())
			 );
	(*this)=B;
      }

      template<class T>
	inline void draw(ip_Image<T> & Ima,
			 T color,int Thickness=1){
	ip_BoundingBox C(Ima.nbColumns(),Ima.nbLines());
	int li,co;
	int left, right, top, bottom;

	if(Thickness>1){
	  //C.initPosAndSize(1,1,Ima.nbColumns()-2,Ima.nbLines()-2);
	  C.initPosAndSize(Thickness/2+1,Thickness/2+1,Ima.nbColumns()-Thickness-2,Ima.nbLines()-Thickness-2);
	}
	C.intersection(*this);

	if(C.area()>0){
	  if (Thickness==1)
	  {
	  co=C.lastColumn()-1;
	  for(li=C.firstLine();li<C.lastLine();li++){
	    Ima(li,C.firstColumn())=color;
	    Ima(li,co)=color;
	  }
	  li=C.lastLine()-1;
	  for(co=C.firstColumn();co<C.lastColumn();co++){
	    Ima(C.firstLine(),co)=color;
	    Ima(li,co)=color;
	  }
	  }
	  else
	  {
	    int tmin, tmax;
	    if (Thickness%2==0)
	      tmin = -Thickness/2+1;
	    else
	      tmin = -Thickness/2;
	    tmax = Thickness/2;

	    for(int t=tmin; t<=tmax; t++)
	    {
	      left=C.firstColumn()-t;
	      right=C.lastColumn()+t;
	      for(li=C.firstLine()-t;li<C.lastLine()+t;li++)
	      {
		Ima(li,left)=color;
		Ima(li,right)=color;
	      }
	      top=C.firstLine()-t;
	      bottom=C.lastLine()+t;
	      for(co=C.firstColumn()-t;co<C.lastColumn()+t+1;co++)
	      {
		Ima(top,co)=color;
		Ima(bottom,co)=color;
	      }
	    }
	  }
	  /*
	  if(Thickness>1){
	    int li2,co2=C.lastColumn();
	    co=C.lastColumn()-2;
	    for(li=C.firstLine()-1;li<C.lastLine()+1;li++){
	      Ima(li,C.firstColumn()+1)=color;
	      Ima(li,co)=color;
	      Ima(li,C.firstColumn()-1)=color;
	      Ima(li,co2)=color;
	    }
	    li2=C.lastLine();
	    li=C.lastLine()-2;
	    for(co=C.firstColumn()-1;co<C.lastColumn()+1;co++){
	      Ima(C.firstLine()+1,co)=color;
	      Ima(li,co)=color;
	      Ima(C.firstLine()-1,co)=color;
	      Ima(li2,co)=color;
	    }
	  }
	  */
	}
      }
	

      void scale(float scalefactor)
      {
	m_iFirstColumn = int(m_iFirstColumn*scalefactor+.5);
	m_iFirstLine = int(m_iFirstLine*scalefactor+.5);
	m_iWidth = int(m_iWidth*scalefactor+.5);
	m_iHeight = int(m_iHeight*scalefactor+.5);
      }

      void scale(float width_scale, float height_scale)
      {
	m_iFirstColumn = int(m_iFirstColumn*width_scale+.5);
	m_iFirstLine = int(m_iFirstLine*height_scale+.5);
	m_iWidth = int(m_iWidth*width_scale+.5);
	m_iHeight = int(m_iHeight*height_scale+.5);
      }

      // scales the BB around the centre
      void enlarge(float scalefactor)
      {
	ip_BoundingBox bbtmp;
	bbtmp.m_iFirstColumn = int(centerX()-(float)m_iWidth/2*scalefactor);
	bbtmp.m_iFirstLine = int(centerY()-(float)m_iHeight/2*scalefactor);
	bbtmp.m_iWidth = int(m_iWidth*scalefactor);
	bbtmp.m_iHeight = int(m_iHeight*scalefactor);
	*this = bbtmp;
      }
      
      // scales the BB around the centre
      void enlarge(float scalefactor_x, float scalefactor_y)
      {
	ip_BoundingBox bbtmp;
	bbtmp.m_iFirstColumn = int(centerX()-(float)m_iWidth/2*scalefactor_x);
	bbtmp.m_iFirstLine = int(centerY()-(float)m_iHeight/2*scalefactor_y);
	bbtmp.m_iWidth = int(m_iWidth*scalefactor_x);
	bbtmp.m_iHeight = int(m_iHeight*scalefactor_y);
	*this = bbtmp;
      }

      // scales the height of the BB around the centre
      void enlargeY(float scalefactor)
      {
	ip_BoundingBox bbtmp;
	bbtmp.m_iFirstColumn = m_iFirstColumn;
	bbtmp.m_iFirstLine = int(centerY()-(float)m_iHeight/2*scalefactor);
	bbtmp.m_iWidth = m_iWidth;
	bbtmp.m_iHeight = int(m_iHeight*scalefactor);
	*this = bbtmp;
      }

      // returns a distance measure normalised by the widths of both BBs
      float normalisedDistance(ip_BoundingBox bb)
      {
	float dx1 = bb.centerX()-centerX();
	float dy1 = bb.centerY()-centerY();
	float dx2 = dx1/bb.m_iWidth;
	float dy2 = dy1/bb.m_iHeight;
	dx1 /= m_iWidth;
	dy1 /= m_iHeight;
	return 0.5*(sqrtf(dx1*dx1+dy1*dy1)+sqrtf(dx2*dx2+dy2*dy2));
      }

      virtual ~ip_BoundingBox() {}



    };
} 



#endif  // IP_BOUNDING_BOX
