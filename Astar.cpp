#include "Astar.h" 


using namespace std;
using namespace arma;

std::vector<AstarPoint*> Astar::AStarfindroad(const arma::Mat<float>& grid, const AstarPoint& start, const AstarPoint& end)
{
	int rows = grid.n_rows;
	int cols = grid.n_cols;

	vector<AstarPoint*> path;
	priority_queue<AstarPoint*, vector<AstarPoint*>, ComparePoint> openList;  // �����б�
	vector<vector<bool>> closed(rows, vector<bool>(cols, false));  // �ر��б�
	vector<vector<bool>> boolopen(rows, vector<bool>(cols, false));  // �����б���ʾ�Ƿ��ڿ����б���

	std::unordered_map<std::pair<int, int>, AstarPoint*, PairHash> xyToAstarPoint;//����(x,y)��openList��Ӧ����(x,y���ṹ��ָ��Ķ�Ӧ��ϵ�������޸�
	// ����ʼ����뿪���б�
	openList.push(new AstarPoint(start.x, start.y));
	boolopen[start.x][start.y] = true;
	xyToAstarPoint.insert({ {start.x, start.y}, new AstarPoint(start.x, start.y) });

	while (!openList.empty()) {//�����б�Ϊ��
		// �ӿ����б���ȡ��������С�Ľڵ�
		AstarPoint* current = openList.top();
		openList.pop();//�ӿ����б���ɾ��
		xyToAstarPoint.erase({ current->x,current->y });

		// �жϵ�ǰ�ڵ��Ƿ�Ϊ�յ㣬���򷵻�·��
		if (current->x == end.x && current->y == end.y) {
			while (current != nullptr) {
				path.push_back(current);
				current = current->parent;
			}
			reverse(path.begin(), path.end());  // ��ת·�����õ�����㵽�յ��˳��
			break;
		}

		// ����ǰ�ڵ���Ϊ�ѷ���
		closed[current->x][current->y] = true;
		boolopen[start.x][start.y] = false;

		// ��ȡ��ǰ�ڵ���Χ�ɴ�ڵ㣬����8����
		for (int i = 0; i < roaddx.size(); ++i) {
			int nextX = current->x + roaddx[i];
			int nextY = current->y + roaddy[i];

			std::pair<int, int> tempnode = { nextX,nextY };
			// ���ýڵ��Ƿ���ͼ�ķ�Χ���Ҳ����ϰ���(Ϊ0),�Ҳ���colse�б���
			// �������򣺲��������ϰ����Ե(���ô�Ȩ)
			// ���������ϰ�������Ӱ�����������Խ�����Ȩ�ؼӴ����ȼ��������
			if (nextX >= 0 && nextX < rows && nextY >= 0 && nextY < cols &&
				 !closed[nextX][nextY]) {
				double nowg = (std::abs(roaddx[i]) + std::abs(roaddy[i]) == 2) ? oblique_gcost : straight_gcost;

				// ����ýڵ㲻��open�б��У�����open�б�
				if (!boolopen[nextX][nextY]) {
					AstarPoint* nextPoint = new AstarPoint(nextX, nextY);
					nextPoint->parent = current;
					nextPoint->g = nowg;  // �ýڵ��g cost

					if (grid(nextX, nextY) == 1)//Ϊ����㣬����Ϊ���Ȩ
					{
						nextPoint->calculateH(end); nextPoint->h = nextPoint->h + 2 * Maxvalue;
					}

					//else if (isneighbor_obstacle(grid, tempnode)) { nextPoint->calculateH(end); nextPoint->h = nextPoint->h + Maxvalue; }//������ǵ���㣬�������ص�8�������е����hֵ��Ϊ��ֵ
					else { nextPoint->calculateH(end); }  // �������������������
					
					nextPoint->calculateF();  // �����ܴ���
					openList.push(nextPoint);//����open�б��в���Ҫdelete,���ͳһdelete
					xyToAstarPoint.insert({ {nextX, nextY}, nextPoint });
					boolopen[nextX][nextY] = true;
				}
				else {//�Ѿ���open�б��У��ж�gֵ�������֮ǰ��С�������g,f�Լ����ڵ�
					auto it = xyToAstarPoint.find({ nextX, nextY });
					if (it != xyToAstarPoint.end()) {
						if (nowg < it->second->g) {
							it->second->parent = current;
							it->second->g = nowg;
							it->second->calculateF();
						}
					}
					else {
						QString line = QString::fromLocal8Bit("�Ҳ�����openList���Ѿ����ڵĵ㣡");
						std::cout << line.toLocal8Bit().data() << std::endl;
						vector<AstarPoint*> emptypath;
						return emptypath;//���ؿ�����
					}

				}
			}
		}
	}

	// �ͷ��ڴ�
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
		QString line = QString::fromLocal8Bit("���������곬�ޣ�");
		std::cout << line.toLocal8Bit().data() << std::endl;
		return false;
	}
	//// ����8����
	//for (size_t i = 0; i < roaddx.size(); ++i) {
	//	int newRow = orirow + roaddx[i];
	//	int newCol = oricol + roaddy[i];

	//	// ����������Ƿ���ͼ��Χ��
	//	if (newRow >= 0 && newRow < mask.n_rows &&
	//		newCol >= 0 && newCol < mask.n_cols &&
	//		mask(newRow, newCol) == 1) {
	//		return true; // ���������д���ֵΪ1������
	//	}
	//}

	//�趨��������İ뾶
	int R_neighbor = 1;
	for (int i = -R_neighbor; i <= R_neighbor; i++) {
		for (int j = -R_neighbor; j <= R_neighbor; j++) {
			int newRow = orirow + i;
			int newCol = oricol + j;

			// ����������Ƿ���ͼ��Χ��
			if (newRow >= 0 && newRow < mask.n_rows &&
				newCol >= 0 && newCol < mask.n_cols &&
				mask(newRow, newCol) == 1) {
				return true; // ���������д���ֵΪ1������
			}
		}
	}
	return false; // δ�ҵ������д���ֵΪ1������
}
