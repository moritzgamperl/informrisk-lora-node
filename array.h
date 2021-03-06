#include <stdio.h>

// function to sort the array in ascending order
int Array_median(long *array , int n)
{ 
    // declare some local variables
    int i=0 , j=0 , temp=0;

    for(i=0 ; i<n ; i++)
    {
        for(j=0 ; j<n-1 ; j++)
        {
            if(array[j]>array[j+1])
            {
                temp        = array[j];
                array[j]    = array[j+1];
                array[j+1]  = temp;
            }
        }
    }
/*
    Serial.println("\nThe array after sorting is..\n");
    for(i=0 ; i<n ; i++)
    {
        Serial.println("\narray_1[%d] : %d",i,array[i]);
    } */

    long median=0;
    
    // if number of elements are even
    if(n%2 == 0)
        median = (array[(n-1)/2] + array[n/2])/2.0;
    // if number of elements are odd
    else
        median = array[n/2];
    
    return median;
    //Serial.print("The median is ");
    //Serial.println(median);
}
