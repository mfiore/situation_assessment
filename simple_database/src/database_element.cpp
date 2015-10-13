/* 
 * File:   DatabaseElement.cpp
 * Author: mfiore
 * 
 * Created on February 2, 2015, 6:26 PM
 */

#include "simple_database/database_element.h"

DatabaseElement::DatabaseElement(string model, string subject, vector<string> predicate, string value) {
    model_=model;
    subject_=subject;
    predicate_=predicate;
    value_=value;
}

DatabaseElement::DatabaseElement(const DatabaseElement& orig) {
    this->model_=orig.model_;
    this->subject_=orig.subject_;
    this->predicate_=orig.predicate_;
    this->value_=orig.value_;
}

DatabaseElement::~DatabaseElement() {
}

