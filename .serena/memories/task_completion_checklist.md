# Task Completion Checklist for noxy

## Before Committing Code
1. **Format Code**: Run `cargo fmt` to ensure consistent formatting
2. **Lint Code**: Run `cargo clippy` and address any warnings
3. **Build Check**: Run `cargo build` to ensure compilation succeeds
4. **Test**: Run `cargo test` if tests exist
5. **Run Application**: Test with `cargo run` to verify functionality

## Code Quality Checks
- Ensure no compiler warnings
- Address clippy suggestions
- Verify error handling is consistent with `anyhow`
- Check that GPU resources are properly managed

## Documentation Updates
- Update `CLAUDE.md` if architectural changes are made
- Add/update doc comments for new public APIs
- Update README.md if user-facing features change

## Performance Considerations
- For graphics code, consider running release builds for performance testing
- Profile GPU usage if making rendering changes
- Test with different window sizes and configurations

## Git Workflow
- Stage relevant changes with `git add`
- Write descriptive commit messages
- Consider running a final test before pushing