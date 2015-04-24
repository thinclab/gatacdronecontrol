//============================================================================
// Name        : PolicyTree.cpp
// Author      : Ekhlas Sonu
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "policytree.h"
using namespace std;

void PolicyTreeNode::readPolicyTreeNode(ifstream& policyfile, int hor){
	this->horizon = hor;

	string fileLine;
	bool lineParsed = false;
	while(!lineParsed && getline(policyfile, fileLine)){
		int hashLocation = fileLine.find("#");
		if(hashLocation > -1){
			fileLine = fileLine.substr(0, hashLocation);
		}
		if(fileLine.empty()){
			continue;
		}
		lineParsed = true;
		char* fileLine1 = (char*) fileLine.c_str();
		vector<string> tokens;
		string delimiter = " \t\n:->";
		Tokenize(fileLine1, tokens, delimiter);

		this->action = atoi(tokens[3].c_str());

		if(horizon > 1){
			for(int obs=0; obs < numObservations; obs++){
				PolicyTreeNode* tempNode = new PolicyTreeNode(numObservations);
				children.push_back(tempNode);
				children[obs]->readPolicyTreeNode(policyfile, hor-1);
			}
		}
	}
}

PolicyTree::PolicyTree(){
	horizon = 0;
	numObservations = 0;
	root = new PolicyTreeNode(numObservations);
}

void PolicyTree::readPolicyTree(string fileName){
	ifstream policyfile(fileName.c_str());

	bool horizonParsed = false;
	bool numObsParsed = false;

	string fileLine;

	while ((!horizonParsed || !numObsParsed) && getline(policyfile, fileLine)){
		int hashLocation = fileLine.find("#");
		if(hashLocation > -1){
			fileLine = fileLine.substr(0, hashLocation);
		}

		if(fileLine.empty()){
			continue;
		}

		char* fileLine1 = (char*) fileLine.c_str();
		vector<string> tokens;
		string delimiter = " \t\n:->";
		Tokenize(fileLine1, tokens, delimiter);

		if(tokens.size()==0){	//Empty Line... Never gonna happen
			continue;
		}

		if(!strcmp(tokens[0].c_str(), "Horizon") || !strcmp(tokens[0].c_str(), "HORIZON")){
			horizon = atoi(tokens[1].c_str());
			horizonParsed = true;
		} else if(!strcmp(tokens[0].c_str(), "Observations") || !strcmp(tokens[0].c_str(), "OBSERVATIONS")){
			numObservations = atoi(tokens[1].c_str());
			root->numObservations = this->numObservations;
			numObsParsed = true;
		}

	}
	if(!horizonParsed){
		cerr<<"No definition of Horizon in file"<<endl;
		return;
	}
	if(!numObsParsed){
		cerr<<"No definition of Observations in file"<<endl;
		return;
	}

	//Now read the policy tree:
	root->readPolicyTreeNode(policyfile, horizon);

}

/*
int main() {
	PolicyTree exampleTree;
	exampleTree.readPolicyTree("src/Policies/Fugitive.policy");
	exampleTree.printTree();
	PolicyTreeNode* rootNode = exampleTree.root;
	cout<<"Action at root = "<<rootNode->action<<endl;
	PolicyTreeNode* nextNode = rootNode->getNextNode(3);
	cout<<"Action at t=2 on obs 3 = "<<nextNode->action<<endl;
	nextNode = nextNode->getNextNode(2);
	cout<<"Action at t=3 on obs 3 followed by 2 = "<<nextNode->action<<endl;
	return 0;
}
*/
