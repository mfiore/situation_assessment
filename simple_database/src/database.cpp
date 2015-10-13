/* 
 * File:   Database.cpp
 * Author: mfiore
 * 
 * Created on February 2, 2015, 6:31 PM
 */

#include "simple_database/database.h"

Database::Database() {
}

Database::Database(const Database& orig) {
}

Database::~Database() {
}

void Database::addElement(DatabaseElement element) {
    database_.push_back(element);

}

vector<DatabaseElement> Database::getElements(DatabaseElement element) {
    vector<DatabaseElement> result;
    for (int i = 0; i < database_.size(); i++) {
        bool good = element.compare(database_[i]);

        if (good == true) {
            result.push_back(database_[i]);
        }
    }
    return result;
}

void Database::removeElement(DatabaseElement element) {
    vector<DatabaseElement> newDatabase;
    for (int i = 0; i < database_.size(); i++) {
        bool found=element.compare(database_[i]);
    	if (!found) {
    		   newDatabase.push_back(database_[i]);
    	}
        else {
        }
    }
    database_ = newDatabase;
}
