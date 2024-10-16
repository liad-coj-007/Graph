#pragma once
#include "Edge.h"
#include <set>
#include <algorithm> // for std::max and std::min
#include <iostream> // For std::cout
#include "Exception/ExceptionGraph/EdgeNotExsit.h"
#include "Exception/ExceptionGraph/VertexNotExist.h"
#include <cmath>
#include <type_traits>
#include <unordered_set>
template<class V,class W>
class Graph{
    public:
    /**
     * @brief return true if we have this edge
     * and otherwise false
     * @param from - the edge we out from
     * @param to - the edge we sign in
     * @return true/false
     */
    bool ContainEdge(const V &from ,const V &to)const{
        
        auto neighbours = graph.find(from);
        if(neighbours == graph.end()){
            return false;
        }
        auto it = neighbours->second.find(to);
        return it != neighbours->second.end();
    }
    
    /**
     * @brief set a new edge to the graph
     * @param from - the edge we out from
     * @param to - the edge we sign in
     * @param weight - the weight of the edge
     */
    void SetEdge(const V &from ,const V &to,const W& weight){
        graph[from][to] = weight;
    }


    /**
     * @brief return a unorderd_map
     *  of vertex that goes parent -> other vertex
     * @param parent - the vertex we want to find who the vertex parent
     * @return a set of sons of parent
     */
    unordered_map<V,W> Parent(const V &parent) const {
        return this->operator[](parent);
    }

    /**
     * @brief return a unorderd_map 
     * of vertex that goes parent -> other vertex
     * @param parent - the vertex we want to find who the vertex parent
     * @return a set of sons of parent
     */
    unordered_map<V,W>& operator[](const V &parent){
        return graph[parent];
    }  

     /**
     * @brief return a unorderd_map 
     * of vertex that goes parent -> other vertex
     * @param parent - the vertex we want to find who the vertex parent
     * @return a set of sons of parent
     */
    const unordered_map<V,W>& operator[](const V &parent) const{
       auto it = graph.find(parent);
       if(it == graph.end()){
            throw VertexNotExist();
       }
       return it->second;    
    }
    


    /**
     * @brief replace edges diractions
     * @return unorderd map of sets
     */
     Graph<V,W> ReplaceDiractions() const{
         Graph<V,W> replacegraph;
         for(auto i = begin(); i != end(); ++i){
            for(const auto &j : i->second){
                replacegraph.SetEdge(j.first,i->first,
                j.second);
            }
         }
         return replacegraph;
    }

    /**
     * @brief get unorderd map of to -> (from,weight)
     */
    unordered_map<V,unordered_map<V,W>> OrderSons()const{
        return ReplaceDiractions().graph;
    }

    /**
     * @brief return the parents of the son vertex
     * @param son - the vertex we search his parents
     * @return a set of parents of the son vertex
     */
    unordered_map<V,W> Son(const V &son) const {
        unordered_map<V,W> sonmap;
        for(auto i = begin();i != end(); ++i){
            auto neigbour = i->second.find(son);
            if(neigbour != i->second.end()){
                sonmap[i->first] = neigbour->second;
            }
        }
        return sonmap;
    }

    /**
     * @brief delete edge from the graph
     * @param from - the vertex we out from
     * @param to - the vertex we sign in to
     */
    void deleteEdge(const V& from,const V& to){
        if(!ContainEdge(from,to)){
            return;
        }

        auto it = getIterator(from,to);
        auto neighbours = graph.find(from);
        neighbours->second.erase(it);
        //if the vertex doesn't connect
        //to save memory
        if(neighbours->second.empty()){
            graph.erase(neighbours);
        }
    }  


    /**
     * @brief return the din(v) of the vertex
     * @param vertex - the vertex we search is deg
     * @return int 
     */
    int degin(const V &vertex)const {
        return Son(vertex).size();
    }

    /**
     * @brief return the dout(v) of the vertex
     * @param vertex - the vertex we search is deg
     * @return int 
     */
    int degout(const V &vertex)const {
        return Parent(vertex).size();
    }

    // /**
    //  * @brief return number of edges on the graph
    //  */
    // int getNumEdges() const{
    //     return graph.size();
    // }


    template<class Function>
    /**
     * @brief building a graph that vertex are the vals in the set
     * @param vertexs the set of vertexs we put on the graph
     * @param function - the function that calculate the weight of edge
     * @param defaultval - check if the user want default val on the graph
     */
    Graph(const set<V> &vertexs,const Function &function,
    const bool defaultin = false){
        for(const auto from : vertexs){
            for(const auto to :vertexs){
                SaveEdge(from,to,function(from,to),defaultin);
            }
        }
    }


    template<class Function>
    /**
     * @brief build a graph that his vertex are
     * v1 - v2
     * @param v1 - vertex 1
     * @param v - vertex 2
     * @param function - function that get the weight
     * of the edges
     */
    Graph(const V &v1 ,const V &v2,const Function function,
    const bool defaultin = false){
        V minvertx = std::min(v1,v2);
        V maxvertx = std::max(v1,v2);
        for(V i = minvertx; i <= maxvertx;i++){
            for(V j = minvertx; j <= maxvertx;j++){
                SaveEdge(i,j,function(i,j),defaultin);
            }
        }
    }

    template<class Function>
    /**
     * @brief build a graph that [V(),maxvertex]
     * @param maxvertex - the max vertex we got
     * @param function - the function calculate the weight
     * @param defaultin - the defualt val we take
     */
    Graph(const V &maxvertex, const Function& function, const bool defaultin = false)
        : Graph(V() + 1, maxvertex, function, defaultin) 
        { // Delegating constructor using initializer list
    }

    class Iterator{
        private:
        const Graph *graph;
        using ExtendedIterator = typename 
        std::unordered_map<V,unordered_map<V,W>>::const_iterator;
        ExtendedIterator extandit;        
        friend class Graph;

          /**
         * @brief constractor of it
         * @param graph -the graph we iterate
         * @param graphit - the it of unorderd map
         */
        Iterator(const Graph *graph,
        ExtendedIterator extandit):
        extandit(extandit),graph(graph){
        }

    public:
        /**
         * @brief operator* it
         * @return pair of edge and weight
         */
        const pair<V,unordered_map<V,W>>& operator*() const {
            return *extandit;
        }

        /**
         * @brief get the next val
         * @return it
         */
        Iterator& operator++(){
            if(extandit == graph->graph.end()){
                return *this;
            }
            ++extandit;
            return *this;
        }

        /**
         * @brief return true if it isn't the same
         * it
         */
        bool operator!=(const Iterator &other) const {
            return extandit != other.extandit;
        }

        /**
        * @brief  operator->
        */
        const std::pair<const V, unordered_map<V, W>>* operator->() const {
            return &(*extandit);
        }

    };


    //default constractor
    Graph() = default;


  
    /**
     * @brief return the allocate bytes of the graph
     * use
     * @return size_t
     */
    size_t memoryUseage() const {
        size_t totalSize = sizeof(graph); 
        for (const auto& outerPair : graph) {
            totalSize += sizeof(outerPair.first) + sizeof(outerPair.second);
            for (const auto& innerPair : outerPair.second) {
                totalSize += sizeof(innerPair.first) + sizeof(innerPair.second);
            }
        }
        return totalSize;
    }

    /**
     * @brief return the weight of the edge we take
     * @param from - the vertex we out
     * @param to - the vertex we sign in
     * @return refernce of the weight
     */
    W& operator()(const V& from, const V& to) {
        return graph[from][to];
    }

    /**
     * @brief return the begin it of the graph
     * @return iterator
     */
    Iterator begin() const {
        return Graph::Iterator(this,graph.begin());
    }

    /**
     * @brief return the end it of the graph
     * @return iterator
     */            // For each inner unordered_map, calculate its memory usage

    Iterator end() const{
        return Graph::Iterator(this,graph.end());
    }

    /**
     * @brief return the weight of the edge we take by const
     * therefore the edge must be on the graph
     * @param from - the vertex we out
     * @param to - the vertex we sign in
     * @return refernce of the weight 
     */
    const W& operator()(const V& from, const V& to) const{
        auto it = getIterator(from,to);
        return it->second;
    }

    private:
    unordered_map<V,unordered_map<V,W>> graph;
    using InnerMapIterator = 
    typename std::unordered_map<V, W>::iterator;
    /**
     * @brief return innermap iterator of the edge 
     * from -> to
     * @param from the vertex we out from
     * @param to - the vertex we sign in to
     * @return a unorderdmap iterator
     */
    InnerMapIterator getIterator(const V &from,const V &to){
        auto neighbours = graph.find(from);
        if(neighbours == graph.end()){
            throw EdgeNotExsit();
        }
        auto it = neighbours->second.find(to);

        if(it == neighbours->second.end()){
            throw EdgeNotExsit();
        }

        return it;
    }

    /**
     * @brief check if we need to save this edge on the memory
     * @param from - the edge we out from
     * @param to - the edge we sign in
     * @param weight - the weight of the edge
     */
    void SaveEdge(const V &from,const V &to,const W &weight,bool defaultin){
        if(!defaultin){
            W defaultweight = W();
            if(defaultweight == weight){
                return;
            }
        }
        SetEdge(from,to,weight);
    }

    /**
     * @brief equal to graph and return true if are the same
     * @param graph1 - the first graph we equal
     * @param graph2 - the second graph we equal
     * @return a bool val if they equal
     */
    friend bool operator==(const Graph &graph1, 
    const Graph &graph2){
        return graph1.graph == graph2.graph;
    }


};

