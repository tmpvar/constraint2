if (typeof require !== "undefined") {
  var Constraint2 = require("../constraint2.js");
  var Vec2 = require("vec2");
}

var ok = function(a, msg) { if (!a) throw new Error(msg || "not ok"); };
var eq = function(a, b) { if (a!==b) throw new Error(a + ' !== ' + b); };

function Edge(start, end) {
  this.start = start;
  this.end = end;
}

var nodeId = 0;
function ConstraintNode() {
  var edges = this.edges = [];

  this.id = nodeId++;

  this.link = function(node) {
    edges.push(new Edge(this, node));
    node.edges.push(new Edge(node, this));
  };

  this.collectLocalDof = function() {
    var dof = this.dof || 0;
    for (var i = 0; i<edges.length; i++) {
      dof += 3;
      dof += edges[i].end.dof || 0;
    }
    return dof;
  };

  this.collectDof = function(last) {
    var dof = this.dof || 0;
    for (var i = 0; i<edges.length; i++) {
      if (last !== edges[i].end) {
        dof += 3;
        dof += edges[i].end.collectDof(this) || 0;
      }
    }
    return dof;
  };

  var lastChanged = {};
  this.lastChanged = function(vec, prev) {
    if (typeof vec !== 'undefined') {
      lastChanged.current = vec;
      lastChanged.previous = prev;
    }
    return lastChanged;
  };

  this.trackLastChanged = function(array) {
    for (var i = 0; i<array.length; i++) {
      array[i].change(this.lastChanged);
    }
  };
}

function FixedConstraint(vec) {
  this.vec = vec;
  this.orig = vec.clone();
  this.name = 'fixed';
  this.dof = -3;
  ConstraintNode.call(this);
  ConstraintNode.call(vec);

  this.link(vec);
}

// point is optional
FixedConstraint.prototype.valid = function(point) {
  return (point || this.vec).equal(this.orig);
};

function DistanceConstraint(a, b) {
  this.update(a, b);

  this.name = 'distance';

  ConstraintNode.call(this);

  ConstraintNode.call(a);
  ConstraintNode.call(b);

  this.link(a);
  this.link(b);

  this.dof = -2;
}

DistanceConstraint.prototype.update = function(a, b) {
  this.a = a || this.a;
  this.b = b || this.b;
  this.orig = this.a.distance(this.b);
  this.angle = this.a.subtract(this.b, true).angleTo(Vec2(1, 0));
}

DistanceConstraint.prototype.valid = function(a, b) {
  return (a || this.a).distance((b || this.b)) === this.orig;
};

function AngleConstraint(a, b, common) {
  common = common || Vec2(0, 0);
  this.update(a, b, common);
  this.orig = this.getAngle();

  this.name = 'angle';

  ConstraintNode.call(this);

  this.trackLastChanged([a, b, common]);

  ConstraintNode.call(a);
  ConstraintNode.call(b);
  ConstraintNode.call(common);

  this.link(a);
  this.link(b);
  this.link(common);

  this.dof = -1;
}

AngleConstraint.prototype.getAngle = function() {
  return this.a.subtract(this.common, true).angleTo(this.b.subtract(this.common, true));
};

AngleConstraint.prototype.update = function(a, b, common) {
  this.a = a || this.a;
  this.b = b || this.b;
  this.common = common || this.common;

  if (this.valid()) {
    this.orig = this.getAngle();
  }
};

AngleConstraint.prototype.valid = function(a, b, common) {
  return this.getAngle() === this.orig;
};


var satisfy = function(constraints) {
  for (var i=0; i<constraints.length; i++) {
    var constraint = constraints[i];
    switch (constraint.name) {
      case 'angle':
        var current = constraint.getAngle();
        var orig = constraint.orig;
        if (!constraint.valid()) {
          var current = constraint.getAngle();
          var orig = constraint.orig;

          var adof = constraint.a.collectLocalDof()
          var ddof = adof - constraint.b.collectLocalDof();
          var cdof = constraint.common.collectLocalDof();

          if (adof <= 0 && ddof <= 0) {
            return false;
          }

          // common point is not fixed, translate connected nodes with dof >= 2
          var changed = constraint.lastChanged();
          if (cdof > 0) {
            var d = changed.current.subtract(changed.previous, true);

            var apply = function(node, from) {
              var seen = {};

              seen[node.id] = true;

              node.edges.filter(function(edge) {
                return edge.end.collectLocalDof() >= 2 && edge.end !== changed.current;
              }).map(function(edge) {
                if (!seen[edge.end.id]) {
                  seen[edge.end.id] = true;
                  edge.end.add(d);
                } else {
                  console.log('dup', node)
                }
              });
            }

            apply(constraint);

          // common point is fixed, use it as a pivot
          } else {
            // TODO: this could go bad really easily.
            //       Use previous value
            if (Math.abs(current) - Math.abs(orig) !== 0) {
              var diffb = constraint.b.subtract(constraint.common, true);
              var diffa = constraint.a.subtract(constraint.common, true);

              if (changed.current === constraint.a) {
                constraint.b.set(diffb.rotate(orig).add(constraint.common));
              } else {
                constraint.a.set(diffa.rotate(-orig).add(constraint.common));
              }
            }
          }
          return true;

        } else {
          return true;
        }

      break;

      case 'distance':
        if (!constraint.valid()) {
          var adof = constraint.a.collectDof()
          var ddof = adof - constraint.b.collectDof();

          // over constrained
          // this means that there are no remaining degrees
          // of freedom to perform this operation.
          if (adof <= 0 && ddof <= 0) {
            return false;
          }

          var a = constraint.a, b = constraint.b;

          // b
          if (ddof < 0) {
            console.warn('TODO [distance satisfaction]: b < a');
          } else if (ddof > 0) {
            console.warn('TODO [distance satisfaction]: b > a');

          // both have the same degrees of freedom
          } else {
            // TODO: actually choose what side to move
            if (adof > 0) {
              var diff = constraint.b.subtract(constraint.a, true);
              var angle = Vec2(0, 1).angleTo(diff) || constraint.angle;
              var l = diff.length();

              // handle the case where the points overlap temporarely
              if (l) {
                var ratio = diff.length() / constraint.orig;
                constraint.b.divide(ratio);
              } else {
                diff.set(constraint.orig, 0);
                diff.rotate(angle);

                constraint.b.set(diff.add(constraint.a));
              }
              constraint.update()
            } else {
              //return false;
            }
          }

        }

      break;

      case 'fixed':
        if (!constraint.valid()) {
          return false;
        }
      break;
    }
  }
};


describe('fixed', function() {
  it('allows the endpoint to move freely (startpoint fixed)', function() {
    var line = [Vec2(0, 0), Vec2(10, 0)];

    var constraints = [new FixedConstraint(line[0])];

    line[1].set(-20, 0);

    ok(!constraints[0].valid(Vec2(100, 10)));

    satisfy(constraints);

    eq(line.join(';'), '(0, 0);(-20, 0)');
  });

  it('rejects movement on fixed points', function() {
    var line = [Vec2(0, 0), Vec2(10, 0)];

    var constraints = [
      new FixedConstraint(line[0]),
      new FixedConstraint(line[1])
    ];

    ok(!constraints[0].valid(Vec2(10, 10)));
    ok(!constraints[1].valid(Vec2(0, 10)));

    eq(line.join(';'), '(0, 0);(10, 0)');
  });
});

describe('distance', function() {
  it('moves both unfixed points', function() {
    var line = [Vec2(0, 0), Vec2(0, 10)];

    var constraints = [
      new DistanceConstraint(line[0], line[1])
    ];

    ok(constraints[0].valid());

    line[0].set(0, 10)

    satisfy(constraints);

    eq(line.join(';'), '(0, 10);(0, 20)');
  });

  it('rejects when fixed and pulled out of range', function() {
    var line = [Vec2(0, 0), Vec2(0, 10)];

    var constraints = [
      new FixedConstraint(line[0]),
      new DistanceConstraint(line[0], line[1])
    ];

    ok(!constraints[1].valid(null, Vec2(0, 11)));

    [
      [0, 20, 10],
      [0, -20, -10]
    ].forEach(function(t) {

      line[1].set(t[0], t[1]);
      satisfy(constraints);

      eq(line[1].toString(), '(0, '+ t[2] + ')');
    });
  });

  it('rotates around fixed (start)', function() {
    var line = [Vec2(0, 0), Vec2(0, 10)];

    var constraints = [
      new FixedConstraint(line[0]),
      new DistanceConstraint(line[0], line[1])
    ];

    ok(constraints[1].valid(null, Vec2(10, 0)));

    line[1].set(20, 0);
    satisfy(constraints);

    eq(line[1].toString(), '(10, 0)');
    eq(line[0].toString(), '(0, 0)');
  });

  // TODO: A        B          C
  //       o--------o----------o
  //       |________|__________| -  both lines are fixed
  //
  //  Test 1: Move B along the line
  //  Test 2: Move B perpendicular to the line
  console.log('\n\nWARNING: missing distance tests');
});


describe('angles', function() {

  it('computes the original angle', function() {
    var shared = Vec2(0, 0);
    var line1 = [shared, Vec2(10, 0)];
    var line2 = [shared, Vec2(0, 20)];
    var c = new AngleConstraint(line1[1], line2[1], shared);

    eq(c.getAngle(), Math.PI/2);
  });

  it('rotates around pivot (180)', function() {
    var shared = Vec2(0, 0);
    var line1 = [shared, Vec2(10, 0)];
    var line2 = [shared, Vec2(0, 20)];

    var constraints = [
      new AngleConstraint(line1[1], line2[1], shared),
      new FixedConstraint(shared)
    ];

    line1[1].set(-20, 0);

    constraints[0].update();
    satisfy(constraints);

    eq(line1.join(';'), '(0, 0);(-20, 0)');
    eq(line2.join(';'), '(0, 0);(0, 20)');
  });

  it('rotates around pivot (90)', function() {
    var shared = Vec2(0, 0);
    var line1 = [shared, Vec2(10, 0)];
    var line2 = [shared, Vec2(0, 20)];

    var constraints = [
      new AngleConstraint(line1[1], line2[1], shared),
      new FixedConstraint(shared)
    ];

    line1[1].set(0, 20);

    constraints[0].update();
    satisfy(constraints);

    eq(line1.join(';'), '(0, 0);(0, 20)');
    eq(line2.join(';'), '(0, 0);(-20, 0)');

    // now rotate it back using the other endpoint
    line2[1].set(0, 20);
    constraints[0].update();
    satisfy(constraints);

    eq(line1.join(';'), '(0, 0);(20, 0)');
    eq(line2.join(';'), '(0, 0);(0, 20)');
  });

  it('moves entire assembly when not fixed', function() {
    var shared = Vec2(0, 0);
    var line1 = [shared, Vec2(10, 0)];
    var line2 = [shared, Vec2(0, 20)];

    var constraints = [
      new AngleConstraint(line1[1], line2[1], line1[0]),
    ];

    line2[1].set(-20, 20);
    constraints[0].update();
    satisfy(constraints);

    eq(shared.toString(), '(-20, 0)')
    eq(line1[1].toString(), '(-10, 0)')
    eq(line2[1].toString(), '(-20, 20)')

  });
});