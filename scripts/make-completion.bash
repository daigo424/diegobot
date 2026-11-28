# Tab completion for this repository's make variables, on top of the usual make target completion:
#   DATASET=   recorded VLA datasets under data/vla/raw
#   EPISODE=   episode numbers of the DATASET given earlier on the line (FAILED=1 for the failed ones)
#   SIM_DATASET=  simulator datasets (HDF5) under data/vla/isaac/demos
#   POLICY=    trained policies under data/vla/train
# Enable it by adding to ~/.bashrc:  source <this repository>/scripts/make-completion.bash
# Paths are looked up from the current directory, so run make from the repository root.

_diegobot_make() {
    local cur prev words cword
    _init_completion -n = || return
    local dataset='' failed='' w
    for w in "${words[@]}"; do
        case $w in
            DATASET=*) dataset=${w#DATASET=} ;;
            FAILED=1) failed=_failed ;;
        esac
    done
    # "=" is a word break for bash, so the completed value replaces only the text after it: reply bare values.
    case $cur in
        DATASET=*)
            local names
            names=$(cd data/vla/raw 2>/dev/null && ls -d -- */ 2>/dev/null | sed 's#/$##' | grep -v '_failed$')
            COMPREPLY=($(compgen -W "$names" -- "${cur#DATASET=}"))
            return ;;
        EPISODE=*)
            local numbers
            numbers=$(ls -d "data/vla/raw/$dataset$failed"/episode_* 2>/dev/null | sed 's#.*episode_0*##; s#^$#0#')
            COMPREPLY=($(compgen -W "$numbers" -- "${cur#EPISODE=}"))
            return ;;
        SIM_DATASET=*)
            COMPREPLY=($(compgen -W "$(ls data/vla/isaac/demos/*.hdf5 2>/dev/null)" -- "${cur#SIM_DATASET=}"))
            return ;;
        POLICY=*)
            COMPREPLY=($(compgen -W "$(ls -d data/vla/train/*/checkpoints/last/pretrained_model 2>/dev/null)" \
                -- "${cur#POLICY=}"))
            return ;;
    esac
    _make "$@"
    # Offer the variables themselves next to the targets.
    if [[ $cur != -* && $cur != *=* ]]; then
        COMPREPLY+=($(compgen -W 'DATASET= EPISODE= FAILED=1 SIM_DATASET= POLICY=' -- "$cur"))
        # Leave the cursor right after "=" so the value can be completed next.
        [[ ${#COMPREPLY[@]} -eq 1 && ${COMPREPLY[0]} == *= ]] && compopt -o nospace
    fi
}

if declare -F _init_completion >/dev/null || { [[ -f /usr/share/bash-completion/bash_completion ]] &&
        . /usr/share/bash-completion/bash_completion; }; then
    # make's own completion is loaded lazily; load it now so _make exists to fall back to.
    declare -F _make >/dev/null || . /usr/share/bash-completion/completions/make
    complete -F _diegobot_make make
fi
