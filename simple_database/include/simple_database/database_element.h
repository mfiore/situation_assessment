/* 
 * File:   Databasethis->h
 * Author: mfiore
 *
 *   represents an element used in the database. Elements have a model (which agent has this knowledge)
 *   a subject, a list of predicates and a value. Ex. PR2_ROBOT HUMAN1 distance TAPE 5
 * Created on February 2, 2015, 6:26 PM
 */

#ifndef DATABASEELEMENT_H
#define	DATABASEELEMENT_H

#include <string>
#include <vector>
#include <iostream>
using namespace std;

class DatabaseElement {
public:
    DatabaseElement(string model, string subject, vector<string> predicate, string value);
    DatabaseElement(const DatabaseElement& orig);
    virtual ~DatabaseElement();

    inline bool compare(DatabaseElement other){
    	bool good=true;
        if (this->model_ != "" && this->model_ != other.model_) {
            good = false;
        } else if (this->subject_ != "" && this->subject_ != other.subject_) {
            good = false; }
        // } else if (this->value_.size()!=other.size() && this->value_.size()!=0) {
            // good = false;
        // }
            else if (this->value_ != "" && this->value_ != other.value_) {
                good=false;
            }
        else if (this->predicate_.size()!=other.predicate_.size() && this->predicate_.size()!=0) {
            good=false;
        }
        else {
        	for (int j=0; j<this->predicate_.size();j++) {
        		if (this->predicate_[j]!=other.predicate_[j]) {
        			good=false;
        		}
        	}
            // if (good) {
            //     for (int j=0; j<this->value.size();j++) {
            //         if (this->value_[j]!=other.value_[j]) {
            //             good=false;
            //         }
            //     }
            // }
        }
        return good;
    }

    string model_;
    string subject_;
    vector<string> predicate_;
    // vector<string>  value_;
    string value_;
private:


};

#endif	/* DATABASEELEMENT_H */

