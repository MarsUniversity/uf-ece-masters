#include "compression.h"

#if 0

extern int errno;

void jpeg_buff_dest(){
}


int compressBuf(unsigned char *bufin, unsigned char *bufout,
								int width, int height, int depth) {
	
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	int i;
	unsigned char *line = bufout;
                    

	#if 0
	line = malloc ((width * depth) * sizeof (char)); // linha
	
	if (!line) {
		fprintf (stderr," Erro ao alocar (line) \n");
		return -1;
	}
	#endif
	
	//fscanf (infile, "%s\n%d %d %s", buf, &x, &y, buf);
	//fprintf (stderr, "%s %d %d %s", buf, x, y, buf);
	//width=x;
	//height=y;
		
	
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);
	jpeg_buff_dest(&cinfo, outfile);
	cinfo.image_width = width;
	cinfo.image_height = height;
	            
	cinfo.input_components = depth;
	
	if (depth == 1) { 
		cinfo.in_color_space = JCS_GRAYSCALE;
	}
	else {
		cinfo.in_color_space = JCS_RGB;
	}
		
	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, JPEG_QUALITY, TRUE);
	jpeg_start_compress(&cinfo, TRUE);
	
		        
	                                   
	for (i = 0; i < height; i++) {
		memset (line, 0, (width*depth)); // clear output buffer line
		//fread (line, (width*depth), 1, infile);
		jpeg_write_scanlines(&cinfo, &line, 1);
	}
	     	                                    
	jpeg_finish_compress(&(cinfo));
	jpeg_destroy_compress(&(cinfo));
	
	//fclose (infile);
	//fclose (outfile);
		                                                       
	return 0;
}

#endif
