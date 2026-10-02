"""Genuine incremental D*-Lite over a four-connected weighted grid."""
import heapq,math
INF=float('inf')
class DStarLite:
 def __init__(self,width,height,costs,start,goal):
  self.width,self.height=width,height;self.costs=list(costs);self.start=start;self.last=start;self.goal=goal;self.km=0.;self.g={};self.rhs={goal:0.};self.queue=[];self.entries={};self.expanded=0;self._push(goal)
 def h(self,a,b):return abs(a[0]-b[0])+abs(a[1]-b[1])
 def value(self,table,node):return table.get(node,INF)
 def key(self,node):
  best=min(self.value(self.g,node),self.value(self.rhs,node));return (best+self.h(self.start,node)+self.km,best)
 def _push(self,node):
  key=self.key(node);self.entries[node]=key;heapq.heappush(self.queue,(key,node))
 def _pop(self):
  while self.queue:
   key,node=heapq.heappop(self.queue)
   if self.entries.get(node)==key:del self.entries[node];return key,node
  return None,None
 def top_key(self):
  while self.queue and self.entries.get(self.queue[0][1])!=self.queue[0][0]:heapq.heappop(self.queue)
  return self.queue[0][0] if self.queue else (INF,INF)
 def neighbours(self,node):
  r,c=node
  for rr,cc in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)):
   if 0<=rr<self.height and 0<=cc<self.width:yield (rr,cc)
 def cost(self,a,b):
  value=self.costs[b[0]*self.width+b[1]]
  return INF if value>=100 else 1.+value
 def update_vertex(self,node):
  if node!=self.goal:self.rhs[node]=min((self.cost(node,n)+self.value(self.g,n) for n in self.neighbours(node)),default=INF)
  self.entries.pop(node,None)
  if self.value(self.g,node)!=self.value(self.rhs,node):self._push(node)
 def compute_shortest_path(self,limit=None):
  self.expanded=0;limit=limit or self.width*self.height*20
  while self.top_key()<self.key(self.start) or self.value(self.rhs,self.start)!=self.value(self.g,self.start):
   if self.expanded>=limit:return False
   old,node=self._pop()
   if node is None:return False
   new=self.key(node)
   if old<new:self._push(node)
   elif self.value(self.g,node)>self.value(self.rhs,node):
    self.g[node]=self.value(self.rhs,node)
    for predecessor in self.neighbours(node):self.update_vertex(predecessor)
   else:
    self.g[node]=INF;self.update_vertex(node)
    for predecessor in self.neighbours(node):self.update_vertex(predecessor)
   self.expanded+=1
  return self.value(self.g,self.start)<INF
 def move_start(self,start):self.km+=self.h(self.last,start);self.last=start;self.start=start
 def update_costs(self,changes):
  affected=set()
  for node,value in changes.items():
   self.costs[node[0]*self.width+node[1]]=value;affected.add(node);affected.update(self.neighbours(node))
  for node in affected:self.update_vertex(node)
  return affected
 def path_cost(self,path):return sum(self.cost(a,b) for a,b in zip(path,path[1:])) if path else INF
 def path(self,max_steps=None):
  if self.value(self.g,self.start)==INF:return []
  node=self.start;result=[node];seen={node};max_steps=max_steps or self.width*self.height
  while node!=self.goal and len(result)<=max_steps:
   candidates=[(self.cost(node,n)+self.value(self.g,n),n) for n in self.neighbours(node)];value,node=min(candidates)
   if value==INF or node in seen:return []
   result.append(node);seen.add(node)
  return result if node==self.goal else []
