---
name: semantic-commits
description: Generate and validate semantic commit messages using the format <type>(<scope>) <subject>. Use when writing, checking, or correcting Conventional Commits-style messages.
---

# Semantic Commit Messages

See how a minor change to your commit message style can make you a better programmer.

Format: `<type>(<scope>): <subject>`

The `<scope>` field can be omitted entirely from the commit message format. Do not include empty parentheses.

Never add Claude as co-author or attribution: no `Co-Authored-By: Claude ...` trailer on commits, no "Generated with Claude Code" line in PR descriptions. Applies to committing, pushing, creating PRs, and any other git/GitHub action.

If an invalid `<type>` is provided, respond with an error message that lists the valid types in this file.

## Example

```
feat: add hat wobble
^--^  ^------------^
|     |
|     +-> Summary in present tense.
|
+-------> Type: chore, docs, feat, fix, refactor, style, or test.
```

More Examples:

1. `feat`: new feature for the user, not a change to build tooling.
2. `fix`: bug fix for the user, not a change to build tooling.
3. `docs`: changes to documentation only.
4. `style`: formatting or style-only changes with no production code change.
5. `refactor`: production code restructuring with no user-facing behavior change.
6. `test`: adding or refactoring tests with no production code change.
7. `chore`: maintenance work with no production code change. If a commit does not clearly fit any type above, use `chore` as the default.

References:

- https://www.conventionalcommits.org/
- https://seesparkbox.com/foundry/semantic_commit_messages
- http://karma-runner.github.io/1.0/dev/git-commit-msg.html