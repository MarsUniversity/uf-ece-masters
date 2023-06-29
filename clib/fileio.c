#include "fileio.h"


/////////////// File I/O ///////////////////////////////

/*
 reads in one line at a time and removes tabs and
 repeated spaces. the returned line is terminated
 with '\0'.
 */
int readALine( FILE *fp, char *line, int size, char comment ){
	char c ;
	char c_old = 0;
	int read = 0, index=0 ;
	char commentFlag=0;
	//line[0] = 'A';
	//memset(line,0,size);

	do {
		read = fscanf( fp, "%c", &c ) ;
		if( c == comment ) commentFlag = 1;

		if( (c == ' ' && c == c_old) || c == '\t' );
		else
			line[index++] = c ;
		c_old = c;
	}
	while( (c != '\n') && (read != EOF) && (c != '\0') ) ;

	line[index-1] = '\0' ;

	if (read == EOF)
		return -1;
	else if( commentFlag ){
		line[0] = '\0';
		return index;
	}
	else
		return index;

}


FILE* openFile(const char *file, const char *mode){
	FILE *fp = NULL;

	fp = fopen(file,mode);

	if(fp == NULL){
		printf("ERROR opening file: %s\n",file);
		return NULL;
	}

	return fp;
}


int write2File(char *name,int len){
	FILE *fd;
	int i,j;

	fd = fopen(name,"w");

	/*
	for(i=0;i<DATAFILE_LEN;i++){
		for(j=0;j<(len);j++){
			fprintf(fd,"%3.20f ",data[i][j]);
		}
		fprintf(fd,"\n");
	}
	*/

	return fclose(fd);
}

///////////////////////////////////////////////////////
