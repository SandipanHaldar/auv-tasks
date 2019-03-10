#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <fstream>
//#include <Eigen3/dense>
#include <iostream>

using namespace cv;
using namespace std;


void multiplyMatrices(int firstMatrix[2][], int secondMatrix[2][], int mult[2][], int rowFirst, int columnFirst, int rowSecond, int columnSecond)
{
	int i, j, k;

	// Initializing elements of matrix mult to 0.
	for(i = 0; i < rowFirst; ++i)
	{
		for(j = 0; j < columnSecond; ++j)
		{
			mult[i][j] = 0;
		}
	}

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i < rowFirst; ++i)
	{
		for(j = 0; j < columnSecond; ++j)
		{
			for(k=0; k<columnFirst; ++k)
			{
				mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}
}

void display(int Mat[2][], int rows, int columns)
{
	int i, j;

	for(i = 0; i < rows; ++i)
	{
		for(j = 0; j < columns; ++j)
		{
			cout << Mat[i][j] << " ";
			if(j == columnSecond - 1)
				cout << endl << endl;
		}
	}
}
int main()
{
	int t,i,j;
	cout<<"enter the time interval"<<endl;
	cin>>t;
	VideoCapture cap("3.avi");
	
	if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }


	int X[2][1],Y[2][1];
	int P[2][2],R[2][2];
	/* Fill the matrix Px and Py by the covariance of the velocities and position; and also fill the measurement covariance matrix */

	while(1)
	{
		Mat frame;
        cap >> frame;
  
        if (frame.empty())
           break;
       	imshow( "Frame", frame );
       	/*here take the threshold for determining red colour*/

		int A[2][2],A1[2][2],x[2][1],y[2][1],Xk[2][1],Yk[2][1],Pk[2][2];
		A[0][0]=1;A[0][1]=t;A[1][0]=0;A[1][1]=1;
		A1[0][0]=1;A[0][1]=0;A[1][0]=t;A[1][1]=1;
		int Xkp[2][1];
		multiplyMatrices(A,X,Xkp,2,2,2,1);
		int Ykp[2][1];
		multiplyMatrices(A,Y,Ykp,2,2,2,1);
		int Pkp[2][2],Q[2][2];
		multiplyMatrices(A,P,Q,2,2,2,2);
		multiplyMatrices(Q,A1,Pkp,2,2,2,2);
		Pkp[0][1]=0;Pkp[1][0]=0;
		int K[2][2],H[2][2];
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				H[i][j]=Pkp[i][j]+R[i][j];
			}
		}
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				K[i][j]=Pkp[i][j]/H[i][j];
			}
		}
		/*put the next values of x and y in the matrixes*/
		int h[2][1],K1[2][1];
		h[0][0]=x[0][0]-Xkp[0][0];
		h[1][0]=x[1][0]-Xkp[1][0];
		multiplyMatrices(K,h,K1,2,2,2,1);
		Xk[0][0]=Xkp[0][0]-K1[0][0];
		Xk[1][0]=Xkp[1][0]-K1[1][0];
		h[0][0]=y[0][0]-Ykp[0][0];
		h[1][0]=y[1][0]-Ykp[1][0];
		multiplyMatrices(K,h,K1,2,2,2,1);
		Yk[0][0]=Ykp[0][0]-K1[0][0];
		Yk[1][0]=Ykp[1][0]-K1[1][0];
		for(i=0;i<2;i++)
		{
			for(j=0;j<1;j++)
			{
				X[i][j]=Xk[i][j];
				Y[i][j]=Yk[i][j];
			}
		}
		
		int a[2][2];
		a[0][0]=1-K[0][0];a[0][1]=K[0][1];a[1][1]=1-K[1][1];a[1][0]=K[1][0];
		multiplyMatrices(a,Pkp,Pk,2,2,2,2);
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				P[i][j]=Pk[i][j];
			}
		}
		 char c=(char)waitKey(25);
		 cout<<"press esc to exit"<<endl;
    	if(c==27)
      	break;




	}
	cap.release();
	destroyAllWindows();


}

