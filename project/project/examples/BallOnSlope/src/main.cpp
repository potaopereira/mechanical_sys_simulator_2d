// EXIT_SUCCESS, EXIT_FAILURE
#include <stdlib.h>

#include <fstream>

// BallOnSlopeSymb
#include "BallOnSlope/BallOnSlope.hpp"

int main(
    int argc,
    char* argv[]
){
    if(argc==2){
        char* filename = argv[1];

        std::fstream f;
        f.open(filename, std::ios::out);
        if(!f){
            std::cout << "File " << filename << " not created." << std::endl;
            f.close();
            return EXIT_FAILURE;
        }else{
            BallOnSlopeSymb* tmp;
            try{
                tmp = new BallOnSlopeSymb();
            }
            catch(std::exception const & exc){
                std::cout << exc.what() << std::endl;
                return EXIT_FAILURE;
            }
            tmp->print_functions(f);
            delete tmp;
            f.close();
            return EXIT_SUCCESS;
        }
    
    }
    return EXIT_FAILURE;
}