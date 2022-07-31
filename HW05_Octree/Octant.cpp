#include "Octant.h"
using namespace BTX;
//  Octant
uint Octant::m_uOctantCount = 0;
uint Octant::m_uMaxLevel = 3;
uint Octant::m_uIdealEntityCount = 5;
Octant::Octant(uint a_nMaxLevel, uint a_nIdealEntityCount)
{
	/*
	* This constructor is meant to be used ONLY on the root node, there is already a working constructor
	* that will take a size and a center to create a new space
	*/
	Init();//Init the default values
	m_uOctantCount = 0;
	m_uMaxLevel = a_nMaxLevel;
	m_uIdealEntityCount = a_nIdealEntityCount;
	m_uID = m_uOctantCount;

	m_pRoot = this;
	m_lChild.clear();

	//create a rigid body that encloses all the objects in this octant, it necessary you will need
	//to subdivide the octant based on how many objects are in it already an how many you IDEALLY
	//want in it, remember each subdivision will create 8 children for this octant but not all children
	//of those children will have children of their own

	std::vector<vector3> lMinMax;

	// loop through each object and update lMinMax
	for (int i = 0; i < m_pEntityMngr->GetEntityCount(); i++) {

		// add rigid body's min and max global coords to lMinMax
		lMinMax.push_back(m_pEntityMngr->GetEntity(i)->GetRigidBody()->GetMinGlobal());
		lMinMax.push_back(m_pEntityMngr->GetEntity(i)->GetRigidBody()->GetMaxGlobal());
	}

	// create the rigid body for the surrounding cube
	RigidBody pRigidBody = RigidBody(lMinMax);

	// set up the values of the octant
	m_fSize = pRigidBody.GetHalfWidth().x * 2.0f;
	m_v3Center = pRigidBody.GetCenterLocal();
	m_v3Min = m_v3Center - pRigidBody.GetHalfWidth();
	m_v3Max = m_v3Center + pRigidBody.GetHalfWidth();

	m_uOctantCount++; // when we add an octant we increment the count
	ConstructTree(m_uMaxLevel); // construct the children
}

bool Octant::IsColliding(uint a_uRBIndex)
{
	//Get how many objects there are in the world
	//If the index given is larger than the number of elements in the bounding object there is no collision
	//As the Octree will never rotate or scale this collision is as easy as an Axis Alligned Bounding Box
	//Get all vectors in global space (the octant ones are already in Global)

	// if index is larger than number of elements, there is no collision
	if (a_uRBIndex >= m_pEntityMngr->GetEntityCount()) {
		return false;
	}

	// get min and max coords of rigid body
	vector3 min = m_pEntityMngr->GetEntity(a_uRBIndex)->GetRigidBody()->GetMinGlobal();
	vector3 max = m_pEntityMngr->GetEntity(a_uRBIndex)->GetRigidBody()->GetMaxGlobal();

	// check for an AABB collision
	if (m_v3Min.x > max.x || m_v3Max.x < min.x || m_v3Min.y > max.y || m_v3Max.y < min.y || m_v3Min.z > max.z || m_v3Max.z < min.z) {
		return false;
	}

	// return true if no collision was found
	return true;
}
void Octant::Display(uint a_nIndex, vector3 a_v3Color)
{
	// display the specified octant
	if (m_uID == a_nIndex) {
		m_pModelMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) * glm::scale(vector3(m_fSize)), a_v3Color);
		return;
	}

	// call Display on all the children in the octant
	for (int i = 0; i < m_uChildren; i++) {
		m_pChild[i]->Display(a_nIndex);
	}
}
void Octant::Display(vector3 a_v3Color)
{
	// display the octant
	m_pModelMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) * glm::scale(vector3(m_fSize)), a_v3Color);

	// call Display on all the children in the octant
	for (int i = 0; i < m_uChildren; i++) {
		m_pChild[i]->Display(a_v3Color);
	}
}
void Octant::Subdivide(void)
{
	// if this node has reach the maximum depth return without changes
	if (m_uLevel >= m_uMaxLevel) return;

	// if this node has been already subdivided return without changes
	if (m_uChildren != 0) return;

	//Subdivide the space and allocate 8 children

	m_uChildren = 8;

	// get size of children
	float size = m_fSize / 2.0f;
	float halfWidth = size / 2.0f;
	vector3 center = m_v3Center;

	// create and add children to m_pChild
	int i = 0;

	// use a triple for loop to make sure each positive and negative x, y and z combination are done
	for (int x = -1; x <= 1; x += 2) {
		for (int y = -1; y <= 1; y += 2) {
			for (int z = -1; z <= 1; z += 2) {

				// create new octant
				Octant* o = new Octant(center + vector3(halfWidth * x, halfWidth * y, halfWidth * z), size);

				// assign parent, root and level
				o->m_pParent = this;
				o->m_pRoot = m_pRoot;
				o->m_uLevel = m_uLevel + 1;

				// subdivide again if not at ideal entity count
				if (o->ContainsAtLeast(m_uIdealEntityCount)) {
					o->Subdivide();
				}

				// add o to children
				m_pChild[i] = o;
				i++;
			}
		}
	}
}
bool Octant::ContainsAtLeast(uint a_nEntities)
{
	//You need to check how many entity objects live within this octant
	
	int counter = 0;

	// total number of entities to check
	int total = 0;

	// whether we are checking all entities or just the ones in the octant
	bool checkAllEntities = true;

	// only check entities in m_EntityList if it has at least 1 entity
	int size = m_EntityList.size();
	if (size > 0) {
		total = size;
		checkAllEntities = false;
	}
	// otherwise check all entities
	else {
		total = m_pEntityMngr->GetEntityCount();
	}

	// find the amount of entities that collide with the octant
	for (int i = 0; i < total; i++) {
		// i is its id normally but if we're checking only m_EntityList then we need to get the id from there
		if (IsColliding(checkAllEntities ? i : m_EntityList[i])) {
			counter++;
		}

		// return true if a_nEntities amount + 1 is reached
		if (counter > a_nEntities) {
			return true;
		}
	}

	return false;
}
void Octant::AssignIDtoEntity(void)
{
	//Recursive method
	//Have to traverse the tree and make sure to tell the entity manager
	//what octant (space) each object is at
	
	// traverse the tree
	for (int i = 0; i < m_uChildren; i++) {
		m_pChild[i]->AssignIDtoEntity();
	}

	// when a leaf has been found, assign an ID and dimension to it
	if (m_uChildren == 0) {
		for (int i = 0; i < m_pEntityMngr->GetEntityCount(); i++) {
			if (IsColliding(i)) {
				m_EntityList.push_back(i);
				m_pEntityMngr->AddDimension(i, m_uID);
			}
		}
	}
}
//-------------------------------------------------------------------------------------------------------------------
// You can assume the following is fine and does not need changes, you may add onto it but the code is fine as is
// in the proposed solution.
void Octant::Init(void)
{
	m_uChildren = 0;

	m_fSize = 0.0f;

	m_uID = m_uOctantCount;
	m_uLevel = 0;

	m_v3Center = vector3(0.0f);
	m_v3Min = vector3(0.0f);
	m_v3Max = vector3(0.0f);

	m_pModelMngr = ModelManager::GetInstance();
	m_pEntityMngr = EntityManager::GetInstance();

	m_pRoot = nullptr;
	m_pParent = nullptr;
	for (uint n = 0; n < 8; n++)
	{
		m_pChild[n] = nullptr;
	}
}
void Octant::Swap(Octant& other)
{
	std::swap(m_uChildren, other.m_uChildren);

	std::swap(m_fSize, other.m_fSize);
	std::swap(m_uID, other.m_uID);
	std::swap(m_pRoot, other.m_pRoot);
	std::swap(m_lChild, other.m_lChild);

	std::swap(m_v3Center, other.m_v3Center);
	std::swap(m_v3Min, other.m_v3Min);
	std::swap(m_v3Max, other.m_v3Max);

	m_pModelMngr = ModelManager::GetInstance();
	m_pEntityMngr = EntityManager::GetInstance();

	std::swap(m_uLevel, other.m_uLevel);
	std::swap(m_pParent, other.m_pParent);
	for (uint i = 0; i < 8; i++)
	{
		std::swap(m_pChild[i], other.m_pChild[i]);
	}
}
void Octant::Release(void)
{
	//this is a special kind of release, it will only happen for the root
	if (m_uLevel == 0)
	{
		KillBranches();
	}
	m_uChildren = 0;
	m_fSize = 0.0f;
	m_EntityList.clear();
	m_lChild.clear();
}
void Octant::ConstructTree(uint a_nMaxLevel)
{
	//If this method is tried to be applied to something else
	//other than the root, don't.
	if (m_uLevel != 0)
		return;

	m_uMaxLevel = a_nMaxLevel; //Make sure you mark which is the maximum level you are willing to reach
	m_uOctantCount = 1;// We will always start with one octant

	//If this was initialized before make sure its empty
	m_EntityList.clear();//Make sure the list of entities inside this octant is empty
	KillBranches();
	m_lChild.clear();

	//If we have more entities than those that we ideally want in this octant we subdivide it
	if (ContainsAtLeast(m_uIdealEntityCount))
	{
		Subdivide();
	}
	AssignIDtoEntity();//Add octant ID to Entities
	ConstructList();//construct the list of objects
}
//The big 3
Octant::Octant(vector3 a_v3Center, float a_fSize)
{
	//Will create the octant object based on the center and size but will not construct children
	Init();
	m_v3Center = a_v3Center;
	m_fSize = a_fSize;

	m_v3Min = m_v3Center - (vector3(m_fSize) / 2.0f);
	m_v3Max = m_v3Center + (vector3(m_fSize) / 2.0f);

	m_uOctantCount++;
}
Octant::Octant(Octant const& other)
{
	m_uChildren = other.m_uChildren;
	m_v3Center = other.m_v3Center;
	m_v3Min = other.m_v3Min;
	m_v3Max = other.m_v3Max;

	m_fSize = other.m_fSize;
	m_uID = other.m_uID;
	m_uLevel = other.m_uLevel;
	m_pParent = other.m_pParent;

	m_pRoot, other.m_pRoot;
	m_lChild, other.m_lChild;

	m_pModelMngr = ModelManager::GetInstance();
	m_pEntityMngr = EntityManager::GetInstance();

	for (uint i = 0; i < 8; i++)
	{
		m_pChild[i] = other.m_pChild[i];
	}
}
Octant& Octant::operator=(Octant const& other)
{
	if (this != &other)
	{
		Release();
		Init();
		Octant temp(other);
		Swap(temp);
	}
	return *this;
}
Octant::~Octant() { Release(); };
//Accessors
float Octant::GetSize(void) { return m_fSize; }
vector3 Octant::GetCenterGlobal(void) { return m_v3Center; }
vector3 Octant::GetMinGlobal(void) { return m_v3Min; }
vector3 Octant::GetMaxGlobal(void) { return m_v3Max; }
uint Octant::GetOctantCount(void) { return m_uOctantCount; }
bool Octant::IsLeaf(void) { return m_uChildren == 0; }
Octant* Octant::GetParent(void) { return m_pParent; }
//--- other methods
Octant * Octant::GetChild(uint a_nChild)
{
	//if its asking for more than the 8th children return nullptr, as we don't have one
	if (a_nChild > 7) return nullptr;
	return m_pChild[a_nChild];
}
void Octant::KillBranches(void)
{
	/*This method has recursive behavior that is somewhat hard to explain line by line but
	it will traverse the whole tree until it reaches a node with no children and
	once it returns from it to its parent it will kill all of its children, recursively until
	it reaches back the node that called it.*/
	for (uint nIndex = 0; nIndex < m_uChildren; nIndex++)
	{
		m_pChild[nIndex]->KillBranches();
		delete m_pChild[nIndex];
		m_pChild[nIndex] = nullptr;
	}
	m_uChildren = 0;
}
void Octant::DisplayLeaves(vector3 a_v3Color)
{
	/*
	* Recursive method
	* it will traverse the tree until it find leaves and once found will draw them
	*/
	uint nLeafs = m_lChild.size(); //get how many children this will have (either 0 or 8)
	for (uint nChild = 0; nChild < nLeafs; nChild++)
	{
		m_lChild[nChild]->DisplayLeaves(a_v3Color);
	}
	//Draw the cube
	m_pModelMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) *
		glm::scale(vector3(m_fSize)), a_v3Color);
}
void Octant::ClearEntityList(void)
{
	/*
	* Recursive method
	* It will traverse the tree until it find leaves and once found will clear its content
	*/
	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->ClearEntityList();
	}
	m_EntityList.clear();
}
void Octant::ConstructList(void)
{
	/*
	* Recursive method
	* It will traverse the tree adding children
	*/
	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->ConstructList();
	}
	//if we find a non-empty child add it to the tree
	if (m_EntityList.size() > 0)
	{
		m_pRoot->m_lChild.push_back(this);
	}
}
