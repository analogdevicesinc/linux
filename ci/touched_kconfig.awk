function push(sym){
    if(!(sym in seen)){
        seen[sym]=1
        order[++n]=sym
    }
}

BEGIN{
    inK=0
    curr=""
}

/^diff --git /{
    inK=0
    curr=""
    next
}

/^\+\+\+ b\//{
    path=substr($0,7)
    inK=1
    curr=""
    next
}

!inK { next }

/^@@ /{
    curr=""
    if (match($0, /config[ \t]+([A-Za-z0-9_]+)/, m)) {
        curr=m[1]
    }
    next
}

{
    line=$0
    prefix=substr(line,1,1)
    text=substr(line,2)

    if (prefix==" " || prefix=="+" || prefix=="-") {
	# (menu)?config <symbol>
        if (match(text, /^[ \t]*(menu)?config[ \t]+([A-Za-z0-9_]+)/, m)) {
            curr=m[2]
            if (prefix=="+") { push(curr) }
            next
        }
	# internal to config block
        if ((prefix=="+" || prefix=="-") && curr!="") {
            push(curr)
            next
        }
    }
}

END{
    for (i=1; i<=n; i++) {
        printf "%s%s", order[i], (i<n?OFS:ORS)
    }
}

