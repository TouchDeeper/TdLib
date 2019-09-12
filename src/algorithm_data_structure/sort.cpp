//
// Created by wang on 19-9-9.
//
#include "sort.h"
namespace td{
    /**
     * sort the array ascend
     * @param array_indices store the indices after sort
     * @param array array to be sorted
     * @param left left start point
     * @param right right start point
     */
    void quick_sort_indices(std::vector<int> &array_indices,const std::vector<int> &array,int left ,int right){
        if(left<right)
        {
            int middle;
            double x=array[array_indices[left]];
            int l=left;
            int r=right;
            while(l<r)
            {
                while((array[array_indices[l]]<=x)&&(l<right)) l++ ;
                while((array[array_indices[r]]>x)&&(r>=left)) r-- ;
                if(l<r)
                {
                    int temp = array_indices[l];
                    array_indices[l]=array_indices[r];
                    array_indices[r]=temp ;
                }
            }
            middle=r;
            int temp=array_indices[left];
            array_indices[left]=array_indices[middle];
            array_indices[middle]=temp;

            quick_sort_indices(array_indices,array,left,middle-1);
            quick_sort_indices(array_indices,array,middle+1,right);
        }
    }
}
