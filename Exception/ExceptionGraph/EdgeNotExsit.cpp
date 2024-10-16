#include "EdgeNotExsit.h"
const string msg = "the edge don't exsit";
EdgeNotExsit::EdgeNotExsit():runtime_error(msg){}