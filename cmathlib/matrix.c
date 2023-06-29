
#include "matrix.h"

/*!
	This function initializes a matrix.
	\param r number of rows.
	\param c number of columns.
	\param new_name pointer to a string containing the
	name of the matrix (optional).
	\return out pointer to the new Matrix.
*/
Matrix* initMatrix(int _rows, int _cols, char *new_name){
  int i;

   Matrix *out = ( Matrix*)malloc(sizeof( Matrix));
	#ifdef CHECK
	if(out == NULL)
		printError("ERROR: could not allocate memory for matrix structure %s\n","",' ',out->name,"");
  #endif

  out->rows = _rows;
  out->cols = _cols;

  if(new_name){
  	out->name = (char*)malloc((size_t)strlen(new_name)*sizeof(char));
		#ifdef CHECK
		if(out->name == NULL)
			printError("ERROR: could not allocate memory for %s\n","",' ',out->name,"");
  	#endif
  	strcpy(out->name,new_name);
  }
  else out->name = NULL;

  out->p = (TYPE**)malloc(out->rows*sizeof(TYPE));
  //out->p = (TYPE**)calloc(out->rows,sizeof(TYPE));
	#ifdef CHECK
	if(out->p == NULL)
		printError("ERROR: could not allocate memory for matrix data %s\n","",' ',out->name,"");
  #endif

  for(i=0; i<out->rows; i++){
    out->p[i] = (TYPE*)malloc(out->cols*sizeof(TYPE));
    //out->p[i] = (TYPE*)calloc(out->cols*sizeof(TYPE));
		#ifdef CHECK
		if(out->p[i] == NULL)
			printError("ERROR: could not allocate memory for matrix data %s\n","",' ',out->name,"");
  	#endif
	}

  return out;
}


/*!
	This function initalizes a square identity matrix.  Do not call this
	function and initMatrix() too.
	\param rc row and column size of the new Matrix.
	\return out pointer to the new Matrix.
*/
Matrix* initMatrixEye(int rc){
  int i;
  Matrix *out = ( Matrix*)malloc(sizeof( Matrix));

  out->rows = rc;
  out->cols = rc;
  out->name = NULL;
  out->p = (TYPE**)malloc(out->rows*sizeof(TYPE));
  for(i=0; i<out->rows; i++){
    out->p[i] = (TYPE*)malloc(out->cols*sizeof(TYPE));
    out->p[i][i] = 1.0;
  }
  return out;
}

/*!
	This function takes a matrix and turns it into an identity
	matrix.
	\n in = I
*/
void matrixEye(Matrix *in){
  int i,j;

  for(i=0; i<in->rows; i++){
    for(j=0;j<in->cols; j++){
      if(i==j)
        in->p[i][j] = 1.0;
      else
        in->p[i][j] = 0.0;
    }
  }
}

/*!
	This function creates a Matrix[r,c] of zeros.
	\param r rows.
	\param c columns.
	\return a pointer to a Matrix[r,c]
*/
Matrix* initMatrixZero( int r, int c){
  int i,j;
  Matrix *a = NULL;

  a = (Matrix*)malloc( sizeof(Matrix) );
	if(a == NULL)
		return NULL;

  a->rows = r;
  a->cols = c;
  a->name = NULL;
  a->p = (TYPE**)malloc(a->rows*sizeof(TYPE));
  for(i=0; i<a->rows; i++){
    a->p[i] = (TYPE*)malloc(a->cols*sizeof(TYPE));
    for(j=0;j<a->cols;j++)
      a->p[i][j] = 0.0;
  }

  return a;
}

/*!
	This function creates a diagional square matrix.  Do not call this
	function and initMatrix() or other.

	\todo make this robust aginst a matrix that has already
	been created by initMatrix() or other.
*/
Matrix* initMatrixDiag(int rc, TYPE *dat){
  int i,j;
  Matrix *in = ( Matrix*)malloc(sizeof( Matrix));

  in->rows = rc;
  in->cols = rc;
  in->name = NULL;
  in->p = (TYPE**)malloc(in->rows*sizeof(TYPE));
  for(i=0; i<in->rows; i++){
    in->p[i] = (TYPE*)malloc(in->cols*sizeof(TYPE));
    for(j=0;j<in->cols;j++){
      if(i==j)
        in->p[i][j] = dat[i];
      else
        in->p[i][j] = 0.0;
    }
  }
  return in;
}


/*!
	Free's memory that has been allocated by initMatrix(), matrixDiag(),
	matrixEye(), or other.  It free's the data and the name.
	\param in a pointer to a Matrix to be freed.
 */
void freeMatrix( Matrix *in){
  int i;

  if(in->name) free(in->name);
  if(in->p){
  	for(i=0; i<in->rows; i++)
  		free(in->p[i]); /* delete row memory*/
  	free(in->p);  /* delete row pointers*/
  }
  free(in);
}


/*!
	This function clears a zero matrix.
	\param in a pointer to a Matrix to be set to zero.
*/
void matrixClear( Matrix *in){
  int i,j;
  for(i=0; i<in->rows; i++)
    for(j=0;j<in->cols;j++)
      in->p[i][j] = 0.0;
}

/*!
	Prints a Matrix to the screen.
	\param in a pointer to a matrix.
*/
void printMatrix( Matrix *in){
  int i,j;

  if(in->name) printf("Matrix %s[%ix%i]:\n",in->name,in->rows,in->cols);
  else printf("Matrix[%ix%i]\n",in->rows,in->cols);
  for(i=0;i<in->rows;i++){
    for(j=0;j<in->cols;j++)
      printf(" %f",in->p[i][j]);
    printf("\n");
  }
  printf("\n");
}

/*!
	Fill a matrix from an array
	\param mat a pionter to a Matrix.
	\param dat a pointer to a data array.
*/
void matrixFill( Matrix *mat, TYPE *dat){
  int i,j,k=0;

  for(i=0;i<mat->rows;i++)
    for(j=0;j<mat->cols;j++)
      mat->p[i][j] = dat[k++];
}

/*!
	Fill a column of a matrix from a Vector
*/
void matrixFillColV( Matrix *mat, int j,  Vector *dat){
  int i;

  #ifdef CHECK
  if(j<0 || j>mat->cols) printError
		("Error: matrixFillColV exceeds bounds","",' ',mat->name,dat->name);
  #endif

  for(i=0;i<mat->rows;i++)
  	mat->p[i][j] = dat->p[i];
}


/*!
	Fill a row of a matrix from a Vector
*/
void matrixFillRowV( Matrix *mat, int i,  Vector *dat){
  int j;

  #ifdef CHECK
  if(i<0 || i>mat->rows)
		printError("Error: matrixFillRowV exceeds bounds","",' ',mat->name,dat->name);
  #endif

  for(j=0;j<mat->cols;j++)
  	mat->p[i][j] = dat->p[i];
}


/*!
	Fill a column of a matrix from a Vector
*/
void matrixFillColA( Matrix *mat, int j, TYPE *dat){
  int i;

  #ifdef CHECK
  if(j<0 || j>mat->cols)
		printError("Error: matrixFillColA exceeds bounds",
							 "",' ',mat->name,"scalar");
  #endif

  for(i=0;i<mat->rows;i++)
  	mat->p[i][j] = dat[i];
}


/*!
	Fill a row of a matrix from a Vector
*/
void matrixFillRowA( Matrix *mat, int i, TYPE *dat){
  int j;

  #ifdef CHECK
  if(i<0 || i>mat->rows)
		printError("Error: matrixFillRowA exceeds bounds",
							 "",' ',mat->name,"scalar");
  #endif

  for(j=0;j<mat->cols;j++)
  	mat->p[i][j] = dat[j];
}

/*!
	Add two matricies together: left+right=out
	\param left a pointer to a Matrix.
	\param right a pointer to a Matrix.
	\param out a pointer to a Matrix which contains the return value.
*/
void matrixAdd( Matrix *left,  Matrix *right,  Matrix *out){
  int i,j;

  #ifdef CHECK
  if(left->rows!=right->rows || left->rows!=out->rows || left->cols!=right->cols || left->cols!= out->cols)
    printError("Error: unable to add matricies",
							 left->name,'+',right->name,out->name);
  #endif

  for(i=0;i<left->rows;i++)
    for(j=0;j<left->cols;j++)
      out->p[i][j]=left->p[i][j]+right->p[i][j];
}

/*!
	Subtracts two matricies: left-right=out
	\param left a pointer to a Matrix.
	\param right a pointer to a Matrix.
	\param out a pointer to a Matrix which contains the return value.
*/
void matrixSub( Matrix *left,  Matrix *right,  Matrix *out){
  int i,j;

  #ifdef CHECK
  if(left->rows != right->rows || left->cols != right->cols)
    printError("Error: unable to add matricies",left->name,'-',right->name,out->name);
  #endif

  for(i=0;i<left->rows;i++)
    for(j=0;j<left->cols;j++)
      out->p[i][j]=left->p[i][j]-right->p[i][j];
}

/*!
	Multiplies two matricies together: a*b=c
	\param a pointer to a Matrix.
	\param b pointer to a Matrix.
	\param c pointer to a Matrix which contains the return value.
*/
void matrixMult( Matrix *a,  Matrix *b,  Matrix *c){
	static TYPE sum;
	static int i,j,k;
	#ifdef CHECK
	if (a->cols!=b->rows || a->rows != c->rows || b->cols != c->cols)
		printError("Error: can't multiply matricies",a->name,'*',b->name,c->name);
	#endif

   for(i=0; i<a->rows; i++){
      for(j=0; j<b->cols; j++){
         sum=0;
         for(k=0; k<a->cols; k++){
            sum+=a->p[i][k]*b->p[k][j];
         }
         c->p[i][j]=sum;
      }
   }
}

/*!
	Multiply a matrix by a scalar value.
	\n out = scalar * in
	\param in pointer to a Matrix.
	\param scalar a scalar value of type TYPE.
	\param out a pointer to a Matrix which contains the return value.
*/
void matrixMultS( Matrix *in,TYPE scalar, Matrix *out){
  int i,j;

  for(i=0;i<in->rows;i++)
    for(j=0;j<in->cols;j++)
      out->p[i][j]=scalar*in->p[i][j];
}

/*!
	Perform scalar division on a matrix.
	\n out = in / scalar
	\param in a pointer to a Matrix.
	\param scalar a scalar value of type TYPE.
	\param out a pointer to a Matrix which contains the return value.
*/
void matrixDivS( Matrix *in, TYPE scalar, Matrix *out){
  int i,j;

  for(i=0;i<in->rows;i++)
    for(j=0;j<in->cols;j++)
      out->p[i][j]=in->p[i][j]/scalar;
}


/*!
	Copies the contense of one matrix to another.  The origional matrix
	is not changed during the coping process.  Both matricies must have
	the same dimensions.
	\n b = a
	\param a a pointer to a Matrix.
	\param b a pointer to a Matrix which contains the return value.
*/
void matrixCopy( Matrix *a,  Matrix *b){
	int i,j;
	#ifdef CHECK
	if(a->rows != b->rows || a->cols != b->cols)
		printError("Error: can't set matricies equal to each other","",' ',a->name,b->name);
	#endif
	for(i=0;i<b->rows;i++)
		for(j=0;j<b->cols;j++)
			b->p[i][j] = a->p[i][j];
}

/*!
	This function transposes a matrix.
	\n b = transpose(a).
*/
void matrixTrans( Matrix *a,  Matrix *b){
	int i,j;
	#ifdef CHECK
	if(a->rows != b->cols || a->cols != b->rows)
		printError("Error: can't transpose matrix","",' ',a->name,b->name);
	#endif
	for(i=0;i<b->rows;i++)
		for(j=0;j<b->cols;j++)
			b->p[j][i] = a->p[i][j];
}

/*!
	Inverse Function.
	\n a = inv(a)
	\warning the matrix that is input will be
	destroyed, and the inverse will be put in its place.  Thus,
	if you need to keep the origional matrix data, you should make
	a copy of it and pass the copy to this function.
*/
void matrixInv( Matrix *a, //!< the matrix to be inverted (will be destroyed)
								 Matrix *b //!< a temp matrix which returns the residuals
								){
  int *indxc, *indxr, *ipiv;
  int i, icol, irow, j, k, l,ll;
  TYPE c = 0.0, d = 0.0;
  TYPE big, dum, pivinv, temp = 0.0;
  int n=a->cols;
  int m=a->rows;

  #ifdef CHECK
  if(a->rows != b->rows || a->cols != b->cols || a->rows != a->cols){
  	char *ptr;
  	sprintf(ptr,"Error: unable to invert matrix %s[%i x %i], must be square\n",
    		a->name,a->rows,a->cols);
   	printError(ptr,"",' ',"","");
   }
  #endif

  ipiv = (int*)malloc(sizeof(int)*n); /* ipiv, indxr and indxc are int arrays*/
  indxr = (int*)malloc(sizeof(int)*n); /* used for bookkeeping the pivoting.*/
  indxc = (int*)malloc(sizeof(int)*n);

  for (j=0; j<n; j++) ipiv[j]=0;
  for (i=0; i<n; i++) {		/* main loop over the cols to be reduced*/
    big=0.0;
    for (j=0; j<n; j++)		/* outer loop of the search for a pivot */
      if (ipiv[j] != 1)	/* element*/
	for (k=0; k<n; k++) {
	  if (ipiv[k] == 0) {
	    if (fabs(a->p[j][k]) >= big) {
	      big=fabs(a->p[j][k]);
	      irow=j;
	      icol=k;
	    }
	  }
	  else if (ipiv[k] > 1) {
	    printError("\t\tERROR: gaussj: Singular Matrix-1\n\t\tDisregard solution matrix.\n\n","",' ',a->name,b->name);
	    return;
	  }
	}
    ++(ipiv[icol]);
    if (irow != icol) {
      for (l=0;l<n;l++) SWAP(a->p[irow][l],a->p[icol][l]);
      for (l=0;l<m;l++) SWAP(b->p[irow][l],b->p[icol][l]);
    }
    indxr[i]=irow;  /* This is where the division takes place*/
    indxc[i]=icol;  /* Pivot row gets divided by pivot value*/
    if (a->p[icol][icol] == 0.0) {
      printError("\t\tERROR: gaussj: Singular Matrix-2\n\t\tDisregard solution matrix.\n\n","",' ',a->name,b->name);
      return;
    }
    pivinv=1.0/a->p[icol][icol];
    a->p[icol][icol]=1.0;
    for (l=0;l<n;l++) a->p[icol][l] *= pivinv;
    for (l=0;l<n;l++) b->p[icol][l] *= pivinv;
    for (ll=0;ll<n;ll++)
      if (ll != icol){
	dum=a->p[ll][icol];
	a->p[ll][icol]=0.0;
	for (l=0;l<n;l++) a->p[ll][l] -= a->p[icol][l]*dum;
	for (l=0;l<n;l++) b->p[ll][l] -= b->p[icol][l]*dum;
      }
  }
  for (l=n-1;l>=0;l--) {
    if (indxr[1] != indxc[l])
      for (k=0; k<n;k++)
	SWAP(a->p[k][indxr[l]],a->p[k][indxc[l]]);
  }
  free(ipiv);
  free(indxr);
  free(indxc);
}


/*!
	This function fills a subset of a Matrix.
	\param a Matrix to be filled.
	\param b sub Matrix to put into a
	\param start_row
	\param start_col
*/
void subMatrixFill(Matrix *a, Matrix *b, int start_row, int start_col){
  int i,j;
  /*
  if( (a->rows-start_row)<b->rows || (a->cols-start_col)<b->cols ) {
    printError("sub matrix fill",b->name,'>',a->name,a->name);
    exit(1);
  }
   */
  for(i=0;i<b->rows;i++)
    for(j=0;j<b->cols;j++){
      a->p[start_row+i][start_col+j] = b->p[i][j];
      //printf("%i %i %f\n",i,j,b->p[i][j]);
    }
}

/*!
	This function creates a skew matrix of size 4x4.
	\param a Matrix[4x4] pointer.
	\param b an array of size 3.
*/
void matrixFillSkew4(Matrix *a,TYPE *b){
  if(a->rows != 4 || a->cols != 4){
    printError("skew semetric vector"," ",' ',a->name,"data array");
    exit(1);
  }
  a->p[0][0] = 0.0;
  a->p[0][1] = b[2];
  a->p[0][2] = -b[1];
  a->p[0][3] = b[0];

  a->p[1][0] = -b[2];
  a->p[1][1] = 0.0;
  a->p[1][2] = b[0];
  a->p[1][3] = b[1];

  a->p[2][0] = b[1];
  a->p[2][1] = -b[0];
  a->p[2][2] = 0.0;
  a->p[2][3] = b[2];

  a->p[3][0] = -b[0];
  a->p[3][1] = -b[1];
  a->p[3][2] = -b[2];
  a->p[3][3] = 0.0;
}

/*!
	This creates a skew matrix of size 3x3.
	\param a Matrix[3x3] pointer.
	\param b Vector[3] pointer.
	\note both the Matrix and Vector must already exist.
*/
void matrixFillSkewV(Matrix *a,Vector *b){
  if(a->rows != 3 || a->cols != 3 || b->size != 3){
    printError("skew semetric vector"," ",' ',a->name,b->name);
    exit(1);
  }
  a->p[0][0] = 0.0;
  a->p[0][1] = -b->p[2];
  a->p[0][2] = b->p[1];
  a->p[1][0] = b->p[2];
  a->p[1][1] = 0.0;
  a->p[1][2] = -b->p[0];
  a->p[2][0] = -b->p[1];
  a->p[2][1] = b->p[0];
  a->p[2][2] = 0.0;
}

/*!
	This creates a skew matrix of size 3x3.
	\param a Matrix[3x3] pointer.
	\param b an array of size 3.
	\note the Matrix must already exist.
*/
void matrixFillSkewD(Matrix *a, TYPE *b){
  if(a->rows != 3 || a->cols != 3 ){
    printError("skew semetric double array"," ",' ',a->name,"data array");
    exit(1);
  }
  a->p[0][0] = 0.0;
  a->p[0][1] = -b[2];
  a->p[0][2] = b[1];
  a->p[1][0] = b[2];
  a->p[1][1] = 0.0;
  a->p[1][2] = -b[0];
  a->p[2][0] = -b[1];
  a->p[2][1] = b[0];
  a->p[2][2] = 0.0;
}



/*!
	This allows you to set the name of a Matrix.
	\param a pointer to a Matrix.
	\param b pointer to a character string.
	\param size is the length of the array.
*/
void setNameMatrix( Matrix *a, char *b,int size){
  if(a->name) free(a->name);
  a->name = (char*)malloc((size_t)(size+1)*sizeof(char));
  a->name[size]='\0';
  strcpy(a->name,b);
}

/*************************************************************************/


/*!
	This function initializes the vector.
	\param s size of vector
	\param new_name name of vector (optional)
*/
 Vector* initVector(int s, char *new_name){
	 Vector *in = ( Vector*)malloc(sizeof( Vector));
	#ifdef CHECK
	if(in == NULL)
		printError("ERROR: could not allocate memory for vector sturcture %s\n",
							 "",' ',in->name,"");
  #endif
	in->size = s;
	if(new_name){
		in->name = (char*)malloc((size_t)strlen(new_name)*sizeof(char));
		#ifdef CHECK
		if(in->name == NULL)
			printError("ERROR: could not allocate memory for vector name %s\n",
								 "",' ',in->name,"");
  	#endif
		strcpy(in->name,new_name);
	}
	else in->name = NULL;
	in->p = (TYPE*)malloc((size_t)in->size*sizeof(TYPE));

	#ifdef CHECK
	if(in->p == NULL)
		printError("ERROR: could not allocate memory for data %s\n",
							 "",' ',in->name,"");
  #endif

	return in;
}


/*!
	This function initializes the vector, but does not create
	memory to hold data. It is just a pointer to another vector
	of the same size. That vector is the one that will contain
	the data. This comes in handy when you have a large vector
	that is composed of smaller vectors (i.e. X = [a b c]) and
	you want to be able to use math functions with a, b, and c,
	but also with X.
	\param s size of vector
	\param new_name name of vector (optional)
*/
 Vector* initVectorPointer(int s, char *new_name){
	 Vector *in = ( Vector*)malloc(sizeof( Vector));
   #ifdef CHECK
	 if(in == NULL)
		 printError("ERROR: could not allocate memory for vector sturcture %s\n",
								"",' ',in->name,"");
   #endif
	 in->size = s;
	 if(new_name){
		 in->name = (char*)malloc((size_t)strlen(new_name)*sizeof(char));
     #ifdef CHECK
		 if(in->name == NULL)
			 printError("ERROR: could not allocate memory for vector name %s\n",
									"",' ',in->name,"");
  	 #endif
		 strcpy(in->name,new_name);
	 }
	 else in->name = NULL;

/*  	in->p = (TYPE*)malloc((size_t)in->size*sizeof(TYPE)); */

/*  	#ifdef CHECK */
/*  	if(in->p == NULL) */
/*  		printError("ERROR: could not allocate memory for data %s\n","",' ',in->name,""); */
/*    #endif */

	 return in;
}

/*!
	Add two Vectors together.
	\n out = left + right
*/
void vectorAdd( Vector *left,  Vector *right,  Vector *out){
	int i;
	#ifdef CHECK
	if(left->size != right->size || left->size != out->size)
		printError("Error: can't add vectors",
							 left->name,'+',right->name,out->name);
	#endif
	for(i=0;i<out->size;i++) out->p[i] = left->p[i]+right->p[i];
}

/*!
	This function subtracts a two vectors.  It checks the two input vector
	sizes and the output vector size.
	\n out = left - right
*/
void vectorSub( Vector *left,  Vector *right,  Vector *out){
	int i;
	#ifdef CHECK
	if(left->size != right->size || right->size != out->size)
		printError("Error: can't subtract vectors",
							 left->name,'-',right->name,out->name);
	#endif
	for(i=0;i<out->size;i++)
		out->p[i] = left->p[i]-right->p[i];
}

/*!
	This function multiplies a matrix times a vector and returns a vector.
	The function checks the size of the input and output vectors and the
	matrix.
	\n c = a * b
*/
void vectorMult( Matrix *a,  Vector *b,  Vector *c){
	TYPE sum;
	int i,j,k;
	#ifdef CHECK
	if (a->cols!=b->size || a->rows != c->size)
		printError("Error: can't multiply matrix and vector",
							 a->name,'*',b->name,c->name);
	#endif
	for(i=0; i<a->rows; i++){
  	sum=0;
   	for(k=0; k<a->cols; k++){
  		sum+=a->p[i][k]*b->p[k];
   	}
   	c->p[i]=sum;
	}
}

/*!
	Allows a vector to be multiplied by a scalar number.  The function
	checks the size of the input and output vectors.
	\n out = scalar * in
*/
void vectorMultS( Vector *in, TYPE scalar,  Vector *out){
	int i;
	#ifdef CHECK
	if(in->size != out->size)
		printError("Error: can't scalar mulitiply vectors",
							 in->name,'*',"scalar",out->name);
	#endif
	for(i=0;i<out->size;i++) out->p[i] = scalar*in->p[i];
}

/*!
	Allows a vector to be divided by a scalar number.  The function
	checks the size of the input and output vectors.
	\n out = in / scalar
*/
void vectorDivS( Vector *in, TYPE scalar,  Vector *out){
	int i;
	#ifdef CHECK
	if(in->size != out->size)
		printError("Error: can't scalar divide vectors",
							 in->name,'/',"scalar",out->name);
	#endif
	for(i=0;i<out->size;i++) out->p[i] = in->p[i]/scalar;
}


/*!
	This function fills a vector from an array.  The function
	assumes that the array is of the proper size to completely
	fill the array (ie. no more no less).
	\param a Vector to be filled.
	\param b data array used to fill Vector.
	\note Vector must already exist.
*/
void vectorFill( Vector *a, TYPE *b){
	int i;
	for(i=0;i<a->size;i++) a->p[i] = b[i];
}

/*!
	This function copies the data from one vector to another vector.
	It does not copy the name, only the data.
	\n b = a
*/
void vectorCopy( Vector *a,  Vector *b){
	int i;
	#ifdef CHECK
	if(a->size != b->size)
		printError("Error: can't copy vectors",
							 "",' ',a->name,b->name);
	#endif
	for(i=0;i<b->size;i++) b->p[i] = a->p[i];
}

/*!
	This function sets the contense of a Vector to zero.
*/
void vectorClear( Vector *a){
	int i = 0;
	for(i;i<a->size;i++) a->p[i] = 0.0;
}

/*!
	Allows you to set the name of a vector.  If the vector already
	has a name, then it is deleted and replaced with a the new name.
	The new and old name do not have to be the same size.
*/
void setNameVector( Vector *a, char *b,int size){
  if(a->name) free(a->name);
  a->name = (char*)malloc((size_t)(size+1)*sizeof(char));
  a->name[size]='\0';
  strcpy(a->name,b);
}

/*!
	Free's the memory allocated by a vector, both data and the
	name.
*/
void freeVector( Vector *v){
	if(v->name) free(v->name);
	if(v->p) free(v->p);
	free(v);
}


/*!
	This function copies part of one Vector in to part of another Vector.
	\param t target Vector.
	\param s source Vector.
	\param start_t the start position in the target Vector (remember
	that the data array is 0 based).
	\param start_s the start position in the source Vector.
	\param size the number of elements to be transfered from
	the source Vector to the target Vector.
*/
void subVectorCopy(Vector *t, Vector *s, int start_t, int start_s, int size){
  int i;

  if( (t->size - size)<0 || t->size<start_t || s->size<start_t ){
    //printError
    exit(1);
  }

  for(i=0;i<size;i++)
    t->p[start_t++] = s->p[start_s++];
}

/*!
	Prints a vector
*/
void printVector( Vector *in){
  int i;
  if(in->name) printf("  %s(vector): [",in->name);
  else printf("vector[%i]: [",in->size);
  for(i=0;i<in->size;i++) printf(" %f",in->p[i]);
  printf("]\n");
}

/*!
	Prints a vector to a file designated by a file descriptor.
	\params in Vector to be printed.
	\params fd a file descriptor.
*/
void fprintVector( Vector *in, FILE *fd){
  int i;
  for(i=0;i<in->size;i++) fprintf(fd," %f",in->p[i]);
}

/*!
	Performs the dot product on two Vectos.
*/
TYPE vectorDot(Vector *a, Vector *b){
	int i;
	TYPE out;

	if(a->size != b->size){
		//print error
		//exit
	}
	for(i=0;i<a->size;i++)
		out = a->p[i]*b->p[i];

	return out;
}


////////////////////////////////////////////////////////////////
/*!
	Prints an error message. The format is:
	\n errmsg for: left op right = result.
	\param errmsg error message to print to screen
	\param left name of left data
	\param op operator being used
	\param right name of right data
	\param result name of result data
*/
void printError(char *errmsg,char *left, char op, char *right, char *result){
  fprintf(stderr,errmsg);
  fprintf(stderr," for: %s %c %s = %s ",left,op,right,result);
  fprintf(stderr,"\n");
	exit(1);
}

///////////////////////////////////////////////////////////////


