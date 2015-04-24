/*
 * policytree.h
 *
 *  Created on: Apr 9, 2015
 *      Author: ekhlas
 */

#ifndef POLICYTREE_H_
#define POLICYTREE_H_

#include<vector>
#include<iostream>
#include<fstream>
#include<cstdlib>
#include<string>
#include<cstring>


using namespace std;

struct PolicyTreeNode{
	int numObservations;
	int horizon;
	int action;
	vector<PolicyTreeNode*> children;

	void Tokenize(char* str1, vector<string>& tokens, const string& delimiters = " \t\n:"){
		tokens.clear();
		string str = str1;
		char* tok;
		tok = strtok( (char*) (((((str.c_str()))))), delimiters.c_str());
		while (tok != NULL) {
			tokens.push_back((string) tok);
			tok = strtok(NULL, delimiters.c_str());
		}
	}

	PolicyTreeNode(int numObs){
		numObservations = numObs;
		horizon = -1;
		action = -1;
	}

	void readPolicyTreeNode(ifstream& policyfile, int hor);

	int getAction(){
		return action;
	}

	PolicyTreeNode* getNextNode(int obs){
		if(children.empty() || obs < 0 || obs > numObservations){
			return NULL;
		}
		else return children[obs];
	}

	void printNode(int numTabSpace=1){
		cout<<"act "<<action<<endl;
		if(horizon > 1){
			if(children.empty()){
				cerr<<"Partial policy read."<<endl;
				return;
			}
			for(int obs=0; obs < numObservations; obs++){
				for(int tSp=0; tSp < numTabSpace; tSp++){
					cout<<"\t";
				}
				cout<<"obs "<<obs<<" -> ";
				children[obs]->printNode(numTabSpace+1);
			}
		}
	}

	~PolicyTreeNode(){
		for(unsigned int obs=0; obs < children.size(); obs++){
			if(children[obs] != NULL){
				delete children[obs];
			}
		}
	}
};

struct PolicyTree{
	PolicyTreeNode* root;
	unsigned int horizon;
	unsigned int numObservations;

	void Tokenize(char* str1, vector<string>& tokens, const string& delimiters = " \t\n:"){
		tokens.clear();
		string str = str1;
		char* tok;
		tok = strtok( (char*) (((((str.c_str()))))), delimiters.c_str());
		while (tok != NULL) {
			tokens.push_back((string) tok);
			tok = strtok(NULL, delimiters.c_str());
		}
	}

	PolicyTree();
	void readPolicyTree(string fileName);

	void printTree(){
		cout<<"HORIZON: "<<horizon<<endl;
		cout<<"OBSERVATIONS: "<<numObservations<<endl;
		cout<<"Vector 0: -> ";
		root->printNode();
	}

	~PolicyTree(){
		if(root != NULL){
			delete root;
		}
	}
};


#endif /* POLICYTREE_H_ */
