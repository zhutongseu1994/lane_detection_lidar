#include"type.h"

namespace skywell
{



/**
* @brief Init_KalmanInfo   初始化滤波器的初始值
* @param info  滤波器指针
* @param Q 预测噪声方差 由系统外部测定给定
* @param R 测量噪声方差 由系统外部测定给定
*/
void Init_KalmanInfo(KalmanInfo* info, double Q, double R)
{
	info->A = 1;  //标量卡尔曼
	info->H = 1;  //
	info->P = 10;  //后验状态估计值误差的方差的初始值（不要为0问题不大）
	info->Q = Q;    //预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
	info->R = R;    //测量（观测）噪声方差 可以通过实验手段获得
	info->filterValue = 0;// 测量的初始值
}

double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement)
{
	//预测下一时刻的值
	double predictValue = kalmanInfo->A* kalmanInfo->filterValue;   //x的先验估计由上一个时间点的后验估计值和输入信息给出，此处需要根据基站高度做一个修改
	
	//求协方差
	kalmanInfo->P = kalmanInfo->A*kalmanInfo->A*kalmanInfo->P + kalmanInfo->Q;  //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q
	double preValue = kalmanInfo->filterValue;  //记录上次实际坐标的值
 
	//计算kalman增益
	kalmanInfo->kalmanGain = kalmanInfo->P*kalmanInfo->H / (kalmanInfo->P*kalmanInfo->H*kalmanInfo->H + kalmanInfo->R);  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
	//修正结果，即计算滤波值
	kalmanInfo->filterValue = predictValue + (lastMeasurement - predictValue)*kalmanInfo->kalmanGain;  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
	//更新后验估计
	kalmanInfo->P = (1 - kalmanInfo->kalmanGain*kalmanInfo->H)*kalmanInfo->P;//计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]
	return  kalmanInfo->filterValue;
}


float avgspeed(SpeedArry * speedarry,float speed,uint64_t timestamp)
{
	//printf("speed = %f",speed);
	//printf(",timestamp = %ld\n",timestamp);
	if (speedarry->arry_num < 10)
	{
		if (speedarry->arry_num >= 1)
		{
			float diff_speed = speed - speedarry->speed_arry[speedarry->arry_num -1];
			float diff_timestamp = ((float)(timestamp - speedarry->timestamp_arry[speedarry->arry_num - 1]))/1000;
			float frame_length = diff_timestamp/0.1;
			//printf("arry_num = %d,diff_speed = %f,diff_timestamp = %f,frame_length = %f,%f - %f\n",speedarry->arry_num,diff_speed,diff_timestamp,frame_length,speed,speedarry->speed_arry[speedarry->arry_num -1]);
			speed = speed/frame_length;
			float k = diff_speed/diff_timestamp;
			//if ((k < 20.0)&&(k > -20.0))
			//{
				speedarry->speed_arry[speedarry->arry_num] = speed;
				speedarry->timestamp_arry[speedarry->arry_num] = timestamp;
			/*}else
			{
				if (speedarry->arry_num >= 2)
				{
					printf("+++++++++++++<10++++++++++%f,%f++++++++-K:%f\n",diff_speed,diff_timestamp,k);
					float last_diff_speed = speedarry->speed_arry[speedarry->arry_num -1] - speedarry->speed_arry[speedarry->arry_num -2];
					float last_diff_timestamep = speedarry->timestamp_arry[speedarry->arry_num -1] - speedarry->timestamp_arry[speedarry->arry_num -2];
					speedarry->speed_arry[speedarry->arry_num] = speedarry->speed_arry[speedarry->arry_num -1] + (last_diff_speed/last_diff_timestamep) * diff_timestamp;
					speedarry->timestamp_arry[speedarry->arry_num] = timestamp;
				}else
				{
					speedarry->speed_arry[speedarry->arry_num] = speedarry->speed_arry[speedarry->arry_num -1];
					speedarry->timestamp_arry[speedarry->arry_num] = timestamp;
				}
			}*/
			//printf("\nspeedarry->arry_num = %d,timestamp:%ld\n",speedarry->arry_num - 1,speedarry->timestamp_arry[speedarry->arry_num - 1]);

		}else
		{
			
			speedarry->speed_arry[speedarry->arry_num] = speed;
			speedarry->timestamp_arry[speedarry->arry_num] = timestamp;
			//printf("--->%d,ld\n",speedarry->arry_num,speedarry->timestamp_arry[speedarry->arry_num]);
		}
		speedarry->arry_num = speedarry->arry_num + 1;
		//printf("\nspeedarry->arry_num = %d,timestamp:%ld\n",speedarry->arry_num - 1,speedarry->timestamp_arry[speedarry->arry_num - 1]);

	}else
	{
		for (int i = 0; i < 9 ;i++ )
		{
			speedarry->speed_arry[i] = speedarry->speed_arry[i + 1];
			speedarry->timestamp_arry[i] = speedarry->timestamp_arry[i + 1];
		}
		float diff_speed = speed - speedarry->speed_arry[8];
		float diff_timestamp = ((float)(timestamp -  speedarry->timestamp_arry[8]))/1000;
		float frame_length = diff_timestamp/0.1;
		//printf("arry_num = %d,diff_speed = %f,diff_timestamp = %f,frame_length = %f,%f - %f\n",speedarry->arry_num,diff_speed,diff_timestamp,frame_length,speed,speedarry->speed_arry[8]);
		speed = speed/frame_length;
		float k = diff_speed/diff_timestamp;
		//if ((k < 20.0)&&(k > -20.0))
		//{
			//printf("\n----------------10---------%f,%f-------K:%f\n",diff_speed,diff_timestamp,k);
			speedarry->speed_arry[9] = speed;
			speedarry->timestamp_arry[9] = timestamp;
		/*}else
		{
			
			float last_diff_speed = speedarry->speed_arry[8] - speedarry->speed_arry[7];
			float last_diff_timestamep = speedarry->timestamp_arry[8] - speedarry->timestamp_arry[7];
			speedarry->speed_arry[9] = speedarry->speed_arry[8] + (last_diff_speed/last_diff_timestamep) * diff_timestamp;
			speedarry->timestamp_arry[9] = timestamp;
			printf("\n+++++++++++++++++10++++++++++%f,%f++++++++K:%f  speed_arry[9] = %f\n",diff_speed,diff_timestamp,k,speedarry->speed_arry[9]);
		}*/
	}
	//printf("speedarry->arry_num = %d,timestamp:%ld\n",speedarry->arry_num,speedarry->timestamp_arry[8]);
	//printf("\n");
	//printf("==>");
	float sum = 0;
	for (int i = 0;i < speedarry->arry_num ; i++)
	{
		sum += speedarry->speed_arry[i];
		//printf("%f,",speedarry->speed_arry[i]);
	}
	//printf("\n");
	float avgspeed = sum/(speedarry->arry_num + 1);
	return avgspeed;
};


// 计算 |p1 p2| X |p1 p|
float GetCross(Point& p1, Point& p2,Point& p)
{
	return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
}
//判断点是否在5X5 以原点为左下角的正方形内（便于测试）
/*bool IsPointInMatrix(Point& p,Object &obj)
{ 
	return (GetCross(obj.P[0],obj.P[1],p) * GetCross(obj.P[2],obj.P[3],p) >= 0) && (GetCross(obj.P[1],obj.P[2],p) * GetCross(obj.P[3],obj.P[0],p) >= 0);
	//return false;
}*/


bool IsPointInMatrix(Point& p,Object &obj)
{
        int a = (obj.P[1].x - obj.P[0].x)*(p.y - obj.P[0].y) - (obj.P[1].y - obj.P[0].y)*(p.x - obj.P[0].x);
        int b = (obj.P[2].x - obj.P[1].x)*(p.y - obj.P[1].y) - (obj.P[2].y - obj.P[1].y)*(p.x - obj.P[1].x);
        int c = (obj.P[3].x - obj.P[2].x)*(p.y - obj.P[2].y) - (obj.P[3].y - obj.P[2].y)*(p.x - obj.P[2].x);
        int d = (obj.P[0].x - obj.P[3].x)*(p.y - obj.P[3].y) - (obj.P[0].y - obj.P[3].y)*(p.x - obj.P[3].x);
                if((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0)) {
                        return true;
        }
        return false;
}




}