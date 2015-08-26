/*
 * RingBuffer.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: mfiore
 */

#include "simple_agent_monitor/RingBuffer.h"

RingBuffer::RingBuffer() {
	// TODO Auto-generated constructor stub
	last=0;
	this->size=0;
}

RingBuffer::~RingBuffer() {
	// TODO Auto-generated destructor stub
}

void RingBuffer::allocate(int size) {
	for (int i=0; i<size;i++) {
		T p;
		elements.push_back(p);
	}
	this->size=size;
}

void RingBuffer::insert(T newElement) {
	cout<<"in insert\n";
	last++;
	if (last==size) {
		last=0;
	}
	elements[last]=newElement;

}
vector<T>  RingBuffer::getSequence(int n){
	vector<T> result;

	int nElements=0;
	int i=last;
	while(nElements<n) {
		result.push_back(elements[i]);
		nElements++;

		i--;
		if (i<0) {
			i=n-1;
		}
	}
	return result;

}
