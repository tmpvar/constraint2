var isArray = function (a) {
  return Object.prototype.toString.call(a) === "[object Array]";
};

function Vert(primitive) {
  this._edges = [];
  this.id = Constraint2.id();
  this.primitive = primitive;
}

Vert.prototype.addEdge = function(edge, to) {
  this._edges.push(edge);
  if (to) {
    to._edges.push(edge);
  }
};

Vert.prototype.solveFor = function(coords) {
  var edges = this._edges, l = edges.length;
  for (var i=0; i<l; i++) {
    coords = edges[i].eval(coords);
  }
  return coords;
};

function Edge(constraint, dof) {
  this._constraint = constraint || null;
  this.dof = dof;
}

Edge.prototype.eval = function(coords) {
  return this._constraint(coords);
};

function Constraint2() {
  this._verts = [];
  this._primitives = {};
}

Constraint2.Vert = Vert;
Constraint2.Edge = Edge;

var id = 0;
Constraint2.id = function() {
  return id++;
};

Constraint2.prototype.addConstraint = function(primitives, dof, constraint) {
  if (!isArray(primitives)) {
    primitives = [primitives];
  }

  var l = primitives.length, vert;
  for (var i=0; i<l; i++) {
    if (!primitives[i]._constraintId) {
      vert = new Vert(primitives[i]);
      this._primitives[vert.id] = vert;
      primitives[i]._constraintId = vert.id;
      this._verts.push(vert);
    } else {
      vert = this._primitives[primitives[i]._constraintId];
    }

    vert.addEdge(new Edge(constraint, dof));
  }
};

Constraint2.prototype.position = function(primitive, x, y) {
  var vert = this._primitives[primitive._constraintId];
  var result = vert.solveFor([x, y]);
  vert.primitive.set(result[0], result[1]);
};

if (typeof module !== "undefined" && typeof module.exports == "object") {
  module.exports = Constraint2;
}

if (typeof window !== "undefined") {
  window.Constraint2 = window.Constraint2 || Constraint2;
}

