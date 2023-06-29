
#include "cMathlib.h"

/////////////////////////////////////////////////////////////
/// Constructor cError holds the type of error and a string
/// to identify what the error was.
/// \param a type of error
/// \param msg string holding
/////////////////////////////////////////////////////////////
cMLError::cMLError(int a, const std::string& msg){
	type = a;

	if (msg.size() == 0){
		// strcpy(errMsg," -- no error message -- ");
    errMsg = " -- no error message -- ";
	}
	else {
		// strcpy(errMsg,msg);
    errMsg = msg;
	}
}

/////////////////////////////////////////////////////////////
/// Destructor. This does nothing.
/////////////////////////////////////////////////////////////
cMLError::~cMLError(void){
	;
}


/////////////////////////////////////////////////////////////
/// Prints error to stdout in a format that displays the type
/// of error and the error message.
/////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream &s, cMLError &e){
	s<<"ERROR(";
	switch(e.type){
		case cMLError::FATAL:
			s<<"FATAL";
			break;
		case cMLError::NONFATAL:
			s<<"WARNING";
			break;
	}
	s<<"): "<<e.errMsg<<std::endl;
	return s;
}

/////////////////////////////////////////////////////////////
/// Concatonate two errors together. The highest error type
/// is always kept and concatonates the error strings together.
/////////////////////////////////////////////////////////////
void cMLError::operator+=(cMLError &e){
	type = type > e.type ? type : e.type;
	// strcat(errMsg,"\n");
	// strcat(errMsg,e.errMsg);
  errMsg += "\n" + e.errMsg;
}

/////////////////////////////////////////////////////////////
/// Concatonates the error strings together.
/////////////////////////////////////////////////////////////
void cMLError::operator+=(const std::string& e){
	// strcat(errMsg,e);
  errMsg += e;
}
