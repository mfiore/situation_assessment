/* 
 * File:   Database.h
 * Author: mfiore
 *
 * this class rerepsents a simple database, a vector of elements.
 * Created on February 2, 2015, 6:31 PM
 */

#ifndef DATABASE_H
#define	DATABASE_H

#include <map>
#include <string>
#include <vector>
#include "simple_database/database_element.h"
#include <algorithm>
#include <iostream>
#include "ros/ros.h"


using namespace std;

class Database {
public:
    Database();
    Database(const Database& orig);
    virtual ~Database();

    void addElement(DatabaseElement element);
    //get elements corresponding to the DatabaseElement received in input. Empty fields in this element will be treated as jolly
    vector<DatabaseElement> getElements(DatabaseElement element);
    void removeElement(DatabaseElement element);

    vector<DatabaseElement> database_;

private:



};

#endif	/* DATABASE_H */

