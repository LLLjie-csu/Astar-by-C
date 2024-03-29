#pragma once
/********************************************************************
 * CopyRight (c) Central South University, all rights reserved.
 * Filename: Astar.h
 * Function: A*�㷨������·��Ѱ��
 * Author: ���
 * Version: 1.0
 * Contact:  lijcsu@csu.edu.cn
 * Modified: v1.0, 3/26 2024, �����ļ�.
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

#define Maxvalue 1e6;//����һ������

 // �����ṹ
struct AstarPoint {
	int x, y;  // �������
	double f, g, h;  // f = g + h��gΪ��ʼ�㵽��ǰ��Ĵ��ۣ�hΪ��ǰ�㵽�յ������ʽ����
	AstarPoint* parent;  // ���ڵ�ָ��

	AstarPoint(int _x, int _y) : x(_x), y(_y), f(0), g(0), h(0), parent(nullptr) {}//���캯��

	void calculateH(const AstarPoint& end) {// ����ʽ����,�����и��ľ������Ĺ���
		//h = abs(x - end.x) + abs(y - end.y);
		h = std::sqrt((x - end.x)* (x - end.x)+ (y - end.y)*(y - end.y));
	}

	void calculateF() {// �����ܴ��� f = g + h
		f = g + h;
	}
};
// ����ȽϺ����������ȶ�������
struct ComparePoint {
	bool operator()(AstarPoint* p1, AstarPoint* p2) {
		return p1->f > p2->f;  // �����ܴ��۴�С��������
	}
};

// �Զ����ϣ����
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
	//�жϸõ������������ϰ���
	bool isneighbor_obstacle(const arma::Mat<float>& mask, std::pair<int, int>& oripoint);
private:
	//������ߵķ���(�ܷ�б����)
	std::vector<int> roaddx = { -1, -1, -1, 0, 0, 1, 1, 1 };//������
	std::vector<int> roaddy = { -1, 0, 1, -1, 1, -1, 0, 1 };
	//std::vector<int> roaddx = { -1,0 ,0, 1, };//������
	//std::vector<int> roaddy = { 0, -1,1, 0, };

	double straight_gcost = 1.0;//ֱ��g cost
	double oblique_gcost = 1.0;//б�Խ��ߵ�g cost�趨������g cost������ͬ�����������߶Խ���

};
