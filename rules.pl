submit_rule(S) :-
    gerrit:default_submit(X),
    X =.. [submit | Ls],
    require_signed_off_by(Ls, Nls),
    S =.. [submit | Nls].

require_signed_off_by(S1, S2) :-
    gerrit:commit_message_matches('^Signed-off-by: '),
    gerrit:change_owner(A),
    !,
    S2 = [label('Signed-off-by', ok(A)) | S1].

require_signed_off_by(S1, S2) :-
    gerrit:change_owner(A),
    !,
    S2 = [label('Signed-off-by', reject(A)) | S1].
