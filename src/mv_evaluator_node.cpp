#include "mv_evaluator/mv_evaluator.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "/mv_evaluator/mv_evaluator");

	MVEvaluator mv_evaluator;
	mv_evaluator.executor();

	return 0;
}
