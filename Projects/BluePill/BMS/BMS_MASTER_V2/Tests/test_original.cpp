#include "estimacion_original.h"
#include "data/test_data.h"

int main(){
    float I = test_data[0][1];
    float V1 = test_data[0][1];
    init_estimacion_original(V1,V1);
    
    int num_rows = TEST_DATA_ROWS;
    for(int i = 0; i < num_rows; i++) {
        estimacion_original(test_data[i][1], test_data[i][1], test_data[i][0]);
    }

    return 0;
}