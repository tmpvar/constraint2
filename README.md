# constraint2

## api surface

```javascript

var Constraint2 = require('constraint2')

var c = new Constraint2();
var v = new Vec2(0, 0)

// lets make seg only move vertically
var fn = c.addConstraint(v, function(vec) {
  return new Vec2(v.x, vec.y);
});

// attempt a move
c.position(v, 10, 10);

console.log(v.toString()); // (0, 10);

c.remove(fn);

c.position(v, 10, 10);

console.log(v.toString()); // (10, 10);

```



### license

MIT (see: [license.txt](blob/master/license.txt))
