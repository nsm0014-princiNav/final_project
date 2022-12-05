function DCMbody2nav = qua2dcm(q)

a = q(4);
b = q(1);
c = q(2);
d = q(3);

DCMbody2nav(1,1) = a*a + b*b - c*c - d*d;
DCMbody2nav(1,2) = 2*(b*c - a*d);
DCMbody2nav(1,3) = 2*(a*c + b*d);
DCMbody2nav(2,1) = 2*(a*d + b*c);
DCMbody2nav(2,2) = a*a - b*b + c*c - d*d;
DCMbody2nav(2,3) = 2*(c*d - a*b);
DCMbody2nav(3,1) = 2*(b*d - a*c);
DCMbody2nav(3,2) = 2*(c*d + a*b);
DCMbody2nav(3,3) = a*a - b*b - c*c + d*d;

end
