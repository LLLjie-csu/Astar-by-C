#pragma once
/********************************************************************
 * CopyRight (c) Central South University, all rights reserved.
 * Filename: Astar.h
 * Function: A*算法，最优路径寻找
 * Author: 李杰
 * Version: 1.0
 * Contact:  lijcsu@csu.edu.cn
 * Modified: v1.0, 3/26 2024, 创建文件.
 * Modified: 
 ********************************************************************/

#include <iostream>
#include <vector>
#include <armadillo>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <utility> // for std::pair
#include <QTextCodec>

#define Maxvalue 1e6;//设置一个大数

 // 定义点结构
struct AstarPoint {
	int x, y;  // 点的坐标
	double f, g, h;  // f = g + h，g为起始点到当前点的代价，h为当前点到终点的启发式代价
	AstarPoint* parent;  // 父节点指针

	AstarPoint(int _x, int _y) : x(_x), y(_y), f(0), g(0), h(0), parent(nullptr) {}//构造函数

	void calculateH(const AstarPoint& end) {// 启发式函数,可自行更改距离计算的规则
		//h = abs(x - end.x) + abs(y - end.y);
		h = std::sqrt((x - end.x)* (x - end.x)+ (y - end.y)*(y - end.y));
	}

	void calculateF() {// 计算总代价 f = g + h
		f = g + h;
	}
};
// 定义比较函数用于优先队列排序
struct ComparePoint {
	bool operator()(AstarPoint* p1, AstarPoint* p2) {
		return p1->f > p2->f;  // 按照总代价从小到大排序
	}
};

// 自定义哈希函数
struct PairHash {
	template <class T1, class T2>
	std::size_t operator()(const std::pair<T1, T2>& p) const {
		auto hash1 = std::hash<T1>{}(p.first);
		auto hash2 = std::hash<T2>{}(p.second);
		return hash1 ^ hash2;
	}
};

class Astar
{
public:
	std::vector<AstarPoint*> AStarfindroad(const arma::Mat<float>& mat, const AstarPoint& start, const AstarPoint& end);
	//判断该的领域内有无障碍物
	bool isneighbor_obstacle(const arma::Mat<float>& mask, std::pair<int, int>& oripoint);
private:
	//定义可走的方向(能否斜着走)
	std::vector<int> roaddx = { -1, -1, -1, 0, 0, 1, 1, 1 };//八领域
	std::vector<int> roaddy = { -1, 0, 1, -1, 1, -1, 0, 1 };
	//std::vector<int> roaddx = { -1,0 ,0, 1, };//四领域
	//std::vector<int> roaddy = { 0, -1,1, 0, };

	double straight_gcost = 1.0;//直走g cost
	double oblique_gcost = 1.0;//斜对角线的g cost设定，这里g cost设置相同，这样优先走对角线

};
