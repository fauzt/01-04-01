#include <bits/stdc++.h>
using namespace std;


int main()
{
	srand(time(NULL));
	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
        printf("Forward Ticks:\t\t%d\n", rand()%500);
        printf("Reverse Ticks:\t\t%d\n", rand()%500);
        printf("Left Turn Ticks:\t\t%d\n", rand()%500);
        printf("Right Turn Ticks:\t\t%d\n", rand()%500);
        printf("Forward Distance:\t%d\n", rand()%200);
        printf("Reverse Distance:\t%d\n", rand()%200);
        printf("left turn Distance:\t%d\n", rand()%200);
        printf("Right turn Distance:\t%d\n", rand()%200);
        printf("Object color:\t");
	// if(packet->params[6] == RED)
	// printf("RED\n");
	// else if(packet->params[6] == GREEN)
	printf("GREEN\n");
	// else if (packet->params[6] == WHITE)
	// printf("WHITE\n");
	// else
	// printf("UNKNOWN\n");
        printf("Distance from object in front:\t%d\n", rand()%20);
        //printf("Distance from object on left:\t%d\n", packet->params[9]);
        //printf("Distance from object in right:\t%d\n", packet->params[10]);
        //printf("Angle from reference:\t%d\n\n", packet->params[8]);
	printf("********FAILSAFE STATUS***********TAKE NOTE:\t");
	// if(packet->params[13] == OVER_ON)
	// printf("OFF\n\n");
	// else
	printf("ON\n\n");
        printf("\n---------------------------------------\n\n");
	return 0;
}