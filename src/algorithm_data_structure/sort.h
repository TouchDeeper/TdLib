//
// Created by wang on 19-9-9.
//

#ifndef TDLIB_SORT_H
#define TDLIB_SORT_H

#include <vector>
namespace td{
    /**
     * sort the array ascend
     * @param array_indices store the indices after sort
     * @param array array to be sorted
     * @param left left start point
     * @param right right start point
     */
    void quick_sort_indices(std::vector<int> &array_indices,const std::vector<int> &array,int left ,int right);
}
#endif //TDLIB_SORT_H
