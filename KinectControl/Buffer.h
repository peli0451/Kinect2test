#pragma once
template <class T>
class Buffer
{

private:
	T *array;
	int size;
	int element_count;
	int first_index; //Index der ersten beschriebenen Zelle

public:
	~Buffer() {
		destroy(array);
	}

	Buffer(int _size) {
		array = new T[_size];
		size = _size;
		element_count = 0;
		first_index = 0;
	}

	int begin() {
		return 0;
	}

	int end() {
		return element_count-1;
	}

	int next(int index) {
		return (index + 1) % size;
	}

	void push(T element) {
		array[(first_index + element_count) % size] = element;
		if (element_count < size) {
			element_count++;
		} else {
			first_index = next(first_index);
		}
	}

	T* get(int index) {
		if (element_count == 0) {
			return nullptr;
		}
		return &array[(index + first_index) % size];
	}
};

