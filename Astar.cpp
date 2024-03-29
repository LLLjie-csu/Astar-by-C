#include "Astar.h" 


using namespace std;
using namespace arma;

std::vector<AstarPoint*> Astar::AStarfindroad(const arma::Mat<float>& grid, const AstarPoint& start, const AstarPoint& end)
{
	int rows = grid.n_rows;
	int cols = grid.n_cols;

	vector<AstarPoint*> path;
	priority_queue<AstarPoint*, vector<AstarPoint*>, ComparePoint> openList;  // 开放列表
	vector<vector<bool>> closed(rows, vector<bool>(cols, false));  // 关闭列表
	vector<vector<bool>> boolopen(rows, vector<bool>(cols, false));  // 开放列表，表示是否在开放列表中

	std::unordered_map<std::pair<int, int>, AstarPoint*, PairHash> xyToAstarPoint;//构建(x,y)和openList对应坐标(x,y）结构体指针的对应关系，便于修改
	// 将起始点加入开放列表
	openList.push(new AstarPoint(start.x, start.y));
	boolopen[start.x][start.y] = true;
	xyToAstarPoint.insert({ {start.x, start.y}, new AstarPoint(start.x, start.y) });

	while (!openList.empty()) {//开放列表不为空
		// 从开放列表中取出代价最小的节点
		AstarPoint* current = openList.top();
		openList.pop();//从开放列表中删除
		xyToAstarPoint.erase({ current->x,current->y });

		// 判断当前节点是否为终点，是则返回路径
		if (current->x == end.x && current->y == end.y) {
			while (current != nullptr) {
				path.push_back(current);
				current = current->parent;
			}
			reverse(path.begin(), path.end());  // 反转路径，得到从起点到终点的顺序
			break;
		}

		// 将当前节点标记为已访问
		closed[current->x][current->y] = true;
		boolopen[start.x][start.y] = false;

		// 获取当前节点周围可达节点，遍历8邻域
		for (int i = 0; i < roaddx.size(); ++i) {
			int nextX = current->x + roaddx[i];
			int nextY = current->y + roaddy[i];

			std::pair<int, int> tempnode = { nextX,nextY };
			// 检查该节点是否在图的范围内且不是障碍物(为0),且不在colse列表内
			// 新增规则：不能贴着障碍物边缘(设置大权)
			// 若过遇到障碍物竖穿影像的情况，尝试将地物权重加大，优先级设置最低
			if (nextX >= 0 && nextX < rows && nextY >= 0 && nextY < cols &&
				 !closed[nextX][nextY]) {
				double nowg = (std::abs(roaddx[i]) + std::abs(roaddy[i]) == 2) ? oblique_gcost : straight_gcost;

				// 如果该节点不在open列表中，加入open列表
				if (!boolopen[nextX][nextY]) {
					AstarPoint* nextPoint = new AstarPoint(nextX, nextY);
					nextPoint->parent = current;
					nextPoint->g = nowg;  // 该节点的g cost

					if (grid(nextX, nextY) == 1)//为地物点，设置为最大权
					{
						nextPoint->calculateH(end); nextPoint->h = nextPoint->h + 2 * Maxvalue;
					}

					//else if (isneighbor_obstacle(grid, tempnode)) { nextPoint->calculateH(end); nextPoint->h = nextPoint->h + Maxvalue; }//如果不是地物点，但该像素的8领域内有地物，则h值设为大值
					else { nextPoint->calculateH(end); }  // 正常计算距离启发函数
					
					nextPoint->calculateF();  // 计算总代价
					openList.push(nextPoint);//加入open列表中不需要delete,最后统一delete
					xyToAstarPoint.insert({ {nextX, nextY}, nextPoint });
					boolopen[nextX][nextY] = true;
				}
				else {//已经在open列表中，判断g值，如果比之前的小，则更新g,f以及父节点
					auto it = xyToAstarPoint.find({ nextX, nextY });
					if (it != xyToAstarPoint.end()) {
						if (nowg < it->second->g) {
							it->second->parent = current;
							it->second->g = nowg;
							it->second->calculateF();
						}
					}
					else {
						QString line = QString::fromLocal8Bit("找不到在openList中已经存在的点！");
						std::cout << line.toLocal8Bit().data() << std::endl;
						vector<AstarPoint*> emptypath;
						return emptypath;//返回空容器
					}

				}
			}
		}
	}

	// 释放内存
	while (!openList.empty()) {
		delete openList.top();
		openList.pop();
	}

	return path;
}

bool Astar::isneighbor_obstacle(const arma::Mat<float>& mask, std::pair<int, int>& oripoint)
{
	int orirow = oripoint.first, oricol = oripoint.second;
	if (orirow<0 || orirow>mask.n_rows || oricol<0 || oricol>mask.n_cols)
	{
		QString line = QString::fromLocal8Bit("输入点的坐标超限！");
		std::cout << line.toLocal8Bit().data() << std::endl;
		return false;
	}
	//// 遍历8邻域
	//for (size_t i = 0; i < roaddx.size(); ++i) {
	//	int newRow = orirow + roaddx[i];
	//	int newCol = oricol + roaddy[i];

	//	// 检查新坐标是否在图像范围内
	//	if (newRow >= 0 && newRow < mask.n_rows &&
	//		newCol >= 0 && newCol < mask.n_cols &&
	//		mask(newRow, newCol) == 1) {
	//		return true; // 发现邻域中存在值为1的像素
	//	}
	//}

	//设定遍历领域的半径
	int R_neighbor = 1;
	for (int i = -R_neighbor; i <= R_neighbor; i++) {
		for (int j = -R_neighbor; j <= R_neighbor; j++) {
			int newRow = orirow + i;
			int newCol = oricol + j;

			// 检查新坐标是否在图像范围内
			if (newRow >= 0 && newRow < mask.n_rows &&
				newCol >= 0 && newCol < mask.n_cols &&
				mask(newRow, newCol) == 1) {
				return true; // 发现邻域中存在值为1的像素
			}
		}
	}
	return false; // 未找到邻域中存在值为1的像素
}
