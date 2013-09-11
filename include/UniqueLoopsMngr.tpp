//#include "UniqueLoopsMngr.h"


template <class T>
UniqueLoopsMngr<T>::UniqueLoopsMngr(){
}


/*returns true if correctly added, false if loop already exists*/
template <class T>
bool UniqueLoopsMngr<T>::addLoop(const std::vector<T>& loop){
	bool is_new=!this->loopAlreadyExists(loop);
	if(is_new){
		this->_loops_collection.push_back(loop);
	}
	return is_new;
}

/* returns true if loop already exists in this manager*/
template <class T>
bool UniqueLoopsMngr<T>::loopAlreadyExists(const std::vector<T>& loop){
	bool exists=false;
	for(class std::vector<std::vector<T> >::const_iterator it=this->_loops_collection.begin(); it!=this->_loops_collection.end() ; ++it){
		if(this->compareLoops(*it,loop)){
			exists=true;
			break;
		}
	}
	return exists;
}

/* returns true if equals*/
template <class T>
bool UniqueLoopsMngr<T>::compareLoops(const std::vector<T>& loop1,const std::vector<T>& loop2){
	if(loop1.size() != loop2.size())
		return false;

	//search the first element of loop1 in loop2
	class std::vector<T>::const_iterator it2, common = loop2.begin();
	class std::vector<T>::const_iterator it1=loop1.begin();
	while((common!=loop2.end()) && (*common != loop1.front())){
		common++;
	}

	if(common==loop2.end()){
		return false;
	}
	it2=common;
	bool equal =true;

	for (it1=loop1.begin(); it1!=loop1.end(); ++it1){
		if (*it1!=*it2){
			equal=false;
			break;
		}
		++it2;
		if (it2 == loop2.end()){
			it2=loop2.begin();
		}
	}

	if(!equal){
		it2=common;
		//also check in reverse order
		for (it1=loop1.begin(); it1!=loop1.end(); ++it1){
			if (*it1!=*it2){
				equal=false;
				break;
			}
			if (it2 == loop2.begin()){
				it2=loop2.end();
			}
			--it2;
		}
	}
	return equal;
}
